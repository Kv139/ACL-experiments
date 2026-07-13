##sceenarios, scenes, scene scenario parameters
#take scenioc program, write out to buffer (Scenic to btyes) takes distribution as arg, 

from custom.buffer import Buffer

from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
import gymnasium as gym
from gymnasium import spaces
from typing import Callable

from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario, Scene
from scenic.core.distributions import RejectionException, RandomControlFlowError
from scenic.core.serialization import SerializationError
import gymnasium as gym
from gymnasium import spaces
from typing import Callable
import numpy as np
import random
import scenic

from scenic.core.errors import setDebuggingOptions, InvalidScenarioError

setDebuggingOptions(verbosity=0, fullBacktrace=False, debugExceptions=False, debugRejections=False)



#TODO make ResetException
class ResetException(Exception):
    def __init__(self):
        super().__init__("Resetting")

class CustomMetaDriveEnv(gym.Env):
    """
    verifai_sampler now not an argument added in here, but one specified int he Scenic program
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4} # TODO placeholder, add simulator-specific entries
    
    def __init__(self, 
                 scenario,
                 simulator : Simulator,
                 file: str, 
                 render_mode=None, 
                 max_steps = 1000,
                 observation_space : spaces.Dict = spaces.Dict(),
                 action_space : spaces.Dict = spaces.Dict(),
                 record_scenic_sim_results : bool = True,
                 feedback_fn : callable = lambda x: x,
                 genetic_flag : bool = False, 
                 parameters: list = None,
                 total_timesteps: int = 200000, ##agent steps across all episodes
                 buffer_capacity: int = 20,
                 start_genetic: int = 25
                 ):

        assert render_mode is None or render_mode in self.metadata["render_modes"]

        self.observation_space = observation_space
        self.action_space = action_space
        self.render_mode = render_mode
        self.max_steps = max_steps - 1 # FIXME, what was this about again?
        self.simulator = simulator
        self.scenario = scenario
        self.simulation_results = []

        self.genetic_flag = genetic_flag

        self.feedback_result = None
        self.loop = None
        self.record_scenic_sim_results = record_scenic_sim_results
        self.feedback_fn = feedback_fn

        self.scene_diversity = {} ##for future

        ##custom buffer stuff
        self.category_desc = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19]
        self.k = 10
        self.parameters = {
            "select_road": {"choices": None, "group": "ego"},
            "select_lane": {"choices": None, "group": "ego"},
            "distractor_road": {"choices": None, "group": "distractor"},
            "distractor_lane": {"choices": None, "group": "distractor"},
            "extra_cars": {"choices": [0,1, 2, 3, 4, 5], "group": "extra"},
            "brake_checkers": {"choices": [0, 1, 2, 3], "group": "brake"},
            "tailgaters": {"choices": [0, 1, 2, 3], "group": "tail"},
            "merging_cars": {"choices": [0, 1, 2, 3, 4, 5], "group": "merging"},
            "obstacles": {"choices": [0, 1, 2, 3], "group": "obstacles"},
        }

        self.count_params = [
            "extra_cars", "brake_checkers", "tailgaters", "merging_cars", "obstacles",
        ]


        #self.count_params = ["extra_cars", "trees", "buildings"]

        self.buffer = Buffer(k=self.k, count_params=self.count_params, capacity=buffer_capacity, category_desc=self.category_desc)
        self.start_genetic = start_genetic
        self.total_timesteps = total_timesteps
        self.agent_steps = 0 ##agent steps ACROSS ALL EPISODES, fr tottal_tiemsteps ACROSS ALL EPISODES

        self.episode_counter = 0

        self.curr_full_params=None
        self.curr_bytes = None ##prevoius scenes in bytes
        self.curr_params = None ##params for scenarios
        self.curr_scenario = None ##scenario objects

        self.gae_lambda = 0.95
        self.gamma      = 0.99
        #self.pvl_threshold = 0
        self.replay = False
        self.replay_id = -1

        self.episode_rewards = [] #rewards/values at each step for A2C
        self.episode_values  = []
        self.last_step_truncated = False  # track truncation vs termination for GAE bootstrapping


        self.scenic_file = file
    ##PRINT STATEMENTS
        self.info= {'crossovers': 0, 'mutations': 0, 'replays': 0, 'generations': 0, "param_failures": 0, "scenario_failures": 0, "generation_failures": 0}


    def _make_run_loop(self):
        while True:
            try:
                if self.genetic_flag:           
                    scene = self.get_scene()
                else:
                    scene, _ = self.scenario.generate(feedback=self.feedback_result)
                with self.simulator.simulateStepped(scene, maxSteps=self.max_steps) as simulation:
                    steps_taken = 0
                    self.episode_counter += 1
                    print(f'{self.episode_counter}')
                    # this first block before the while loop is for the first reset call
                    # done() now checks BOTH Scenic's result AND the simulator's own done flag
                    # (MetaDrive can terminate on crash / out-of-road / arrive-dest before Scenic decides)
                    done = lambda: (simulation.result is not None) or simulation.is_done()
                    truncated = lambda: (steps_taken >= self.max_steps) or simulation.get_truncation()
                    observation = simulation.get_obs()
                    info = simulation.get_info() 
                    actions = yield observation, info
                    simulation.actions = actions # TODO add action dict to simulation interfaces

                    while not done():
                        # Probably good that we advance first before any action is set.
                        # this is consistent with how reset works
                        simulation.advance()
                        steps_taken += 1
                        observation = simulation.get_obs()
                        info = simulation.get_info()
                        reward = simulation.get_reward()
                        if done():
                            # record whether the final step was truncation vs termination
                            # (matters for GAE: truncated final steps should bootstrap V, terminated shouldn't)
                            self.last_step_truncated = truncated() and (simulation.result is None)
                            # only run feedback + record when Scenic actually produced a result
                            # (MetaDrive-only terminations leave simulation.result as None)
                            if simulation.result is not None:
                                self.feedback_result = self.feedback_fn(simulation.result)
                                if self.record_scenic_sim_results:
                                    self.simulation_results.append(simulation.result)
                            # simulation.destroy() # FIXME...might redundant?
                            actions = yield observation, reward, done(), truncated(), info
                            break # a little unclean right here
                        actions = yield observation, reward, done(), truncated(), info
                        simulation.actions = actions # TODO add action dict to simulation interfaces
                    
            except ResetException:
                continue

    def reset(self, seed=None, options=None): # TODO will setting seed here conflict with VerifAI's setting of seed?
        # only setting enviornment seed, not torch seed?
        if self.episode_counter > 0:
            self.compute_episode_pvl()
        super().reset(seed=seed)
        self.episode_rewards = []
        self.episode_values  = []
        self.last_step_truncated = False
        if self.loop is None:
            self.loop = self._make_run_loop()
            observation, info = next(self.loop) # not doing self.scene.send(action) just yet
        else:
            observation, info = self.loop.throw(ResetException())
        return observation, info
        
    def step(self, action):
        assert not (self.loop is None), "self.loop is None, have you called reset()?"
        
        self.agent_steps += 1
        observation, reward, terminated, truncated, info = self.loop.send(action)
        return observation, reward, terminated, truncated, info

    def render(self): # TODO figure out if this function has to be implemented here or if super() has default implementation
        """
        likely just going to be something like simulation.render() or something
        """
        # FIXME for one project only...also a bit hacky...
        # self.env.render()
        pass

    def close(self):
        self.simulator.destroy()

    def log_episode_stats(self,reward,value):
        """
        Docstring for log_episode_stats
        
        :param reward: Episode rewards
        :param value: Value estimates from the model
        """
        self.episode_rewards.append(reward)
        self.episode_values.append(value)

    ##update buffer
    def compute_episode_pvl(self):
        """
        Docstring for compute_episode_pvl
        
        :Compute the average postive value loss per episode 
        """
        if len(self.episode_rewards)>= 1 and len(self.episode_values) >= 1:
            lastgaelam = 0 
            advantages = [0] * len(self.episode_rewards) # hold the  
            for t in reversed(range(len(self.episode_rewards))):
                if t == len(self.episode_rewards) - 1:
                    # if truncated (not terminated), bootstrap with V(s_T) rather than 0
                    # -- otherwise we throw away the value estimate at the max_steps boundary
                    if self.last_step_truncated:
                        next_v = self.episode_values[t]
                        nextnonterminal = 1
                    else:
                        next_v = 0
                        nextnonterminal = 0
                else:
                    next_v = self.episode_values[t+1]
                    nextnonterminal = 1
                delta = self.episode_rewards[t] + self.gamma * next_v * nextnonterminal - self.episode_values[t]
                advantages[t] = lastgaelam = delta[0] + self.gamma * self.gae_lambda * nextnonterminal * lastgaelam
                advantages[t] = max(advantages[t],0)        
            
            pvl = np.sum(advantages)/len(advantages)
            if not self.genetic_flag:
                # Compute Sampler feedbach such that values < .1 are > 0, SO sampler tries to find scenes with value > .15
                result = -np.tanh(pvl) + 0.1
                self.feedback_result = result
               # print(f"feedback result was {self.feedback_result} with raw pvl of {pvl}")
                return 
            ##compute pvl, add episode to buffer

            ep_id = self.episode_counter - 1
            if self.replay:
                self.buffer.update(self.replay_id, new_pvl=pvl)
            else:
                self.buffer.add(
                    scene_id=ep_id,
                    full_params=self.curr_full_params,
                    pvl=pvl,
                    params=self.curr_params,
                    scenario=self.curr_scenario,
                    s_bytes=self.curr_bytes
                )
        else:
            ep_id = self.episode_counter-1
            if self.replay:
                self.buffer.update(self.replay_id, new_pvl=0)
            else:
                self.buffer.add(
                    scene_id=ep_id,
                    full_params=self.curr_full_params,
                    pvl=0,
                    params=self.curr_params,
                    scenario=self.curr_scenario,
                    s_bytes=self.curr_bytes
                )
    
    def groups(self):
        groups = {}
        for key, value in self.parameters.items():
            if (value["group"] is not None): 
                g = value["group"]
            else: 
                g = key
            if g not in groups:
                groups[g] = []
            groups[g].append(key)
        return groups
    
    def generate_fresh(self):
        p = {}
        for key, value in self.parameters.items():
            if value["choices"] is not None: 
                p[key] = random.choice(value["choices"])
        return self.generate_scene(params=p)
    
    def get_scene(self):
        """
        Select next training scene:
            case (1): Not enough scenes have been generated: sample a new scene
            case (2): Enough scenes, and even episode: train an a random new or mutated scene
            case (3): Enough scenes, and odd  episode: train on a old scene or sample a new one
        
        TODO : Add a weighted probability calculation for previously seen scenes
        """
        self.replay = False
        if self.episode_counter % 10 == 0:
            print(f"Counts: {self.info}")

        if self.episode_counter >= self.start_genetic and self.episode_counter % 2 == 0: # TODO adjust timing conditions 50/50 exploitation vrs. exploration           
            choice = random.choice([1,2,3])
            if choice == 1: ##crossover
                ret = self.buffer.crossover_same(self.agent_steps, self.total_timesteps)
                if ret is None:
                    scene = self.generate_fresh()
                    self.info['generations'] += 1
                else:
                    s1, s2 = ret
                    s1 = self.read_scene_bytes(s1.scene_id)
                    s2 = self.read_scene_bytes(s2.scene_id)
                    scene = self.crossover_scenes(scene1=s1, scene2=s2)
                    self.info['crossovers'] += 1
            elif choice == 2:
                ret = self.buffer.sample_scene(self.agent_steps, self.total_timesteps)
                if ret is None:
                    scene = self.generate_fresh()
                    self.info['generations'] += 1
                else:
                    s1 = ret
                    s1 = self.read_scene_bytes(s1.scene_id)
                    scene = self.mutate_scene(scene=s1)
                    self.info['mutations'] += 1
                
            else:
                ret = self.buffer.sample_scene(self.agent_steps, self.total_timesteps)
                if ret is None:
                    scene = self.generate_fresh()
                    self.info['generations'] += 1
                else:
                    self.replay = True
                    scene = self.read_scene_bytes(ret.scene_id)
                    self.info['replays'] += 1
                    self.replay_id = ret.scene_id  #you only take items where pvl is above avg pvl in buffer

        else:
            scene = self.generate_fresh()
            self.info['generations'] += 1

        return scene

    def crossover_scenes(self, scene1, scene2):
        """
        Generate a new program with traits from two differnt programs 
        """  
        params = {}
        for i, keys in self.groups().items():
            c = random.choice([scene1, scene2])
            for k in keys:
                if k in c.params:
                    params[k] = c.params[k]
        new_scene = self.generate_scene(params=params)
        return new_scene
    
    def mutate_scene(self,scene):
        """
        Docstring for mutate_scene

        :param scene: Sampled scenic program instance
        Takes a scenic program and randomly chooses certain parameter values
            then condidtions the distribution to them and resamples
            If no valid sample is found returns the original program
        """ 
        groups = self.groups()
        target = random.choice(list(groups.keys()))
        params = {}
        for name, keys in groups.items():
            if name == target:
                continue  
            for k in keys:
                if k in scene.params:
                    params[k] = scene.params[k]
        new_scene = self.generate_scene(params=params)
        return new_scene


    def read_scene_bytes(self, scene_id):
        """
        Docstring for read_scene
        :param scene_bytes: Scenic program written to bytes
        returns: Scene
        """
        scene = self.buffer.get_scene(scene_id)  
        if scene is None:
            self.info["generation_failures"] += 1
            scene, _ = self.scenario.generate()
            self.replay = False
            return scene
        
        if (scene.scenario is not None): 
            fail = "scenario_failures"
        elif scene.params is not None:
            fail = "param_failures"
        else:
            fail = "generation_failures"
        
        try:
            return scene.read_scene(self.scenario, self.scenic_file)
        except (SerializationError, KeyError, RejectionException) as e: ##getting typerror issue
            print(f"failed id was {scene_id}, scenario was genetic {fail}, error type was {e}")
            self.info[fail] += 1
            scene, _ = self.scenario.generate()
            self.replay = False
            return scene

    def generate_scene(self, params={}, objects=None, scene=None):
        """
        Generate a new Scenario 
            (1): If no params are passed generates a new scene from the original program
            (2): If custom params are passed compiles a new program with these values and 
                 saves those params so the program can be reconstructed later
        """
        self.curr_bytes = None ##refresh
        self.curr_params = None
        self.curr_scenario = None


        if objects:
            assert scene is not None

            ##scnenairos
            try: 
                scenario = scenic.scenarioFromFile(self.scenic_file, model="scenic.simulators.metadrive.model",mode2D=True,params={})
            except InvalidScenarioError:
                scenario = self.scenario
            try:
                scenario.conditionOn(scene=scene,objects=objects)
                new_scene, _ = scenario.generate()
                self.curr_scenario = scenario
            except (RejectionException,RandomControlFlowError):
                print(f"Control flow error with objects {objects}")
                for i in range(len(scene.objects)):
                    if i in objects:
                        print(scene.objects[i])

                scenario = self.scenario
                new_scene, _ = scenario.generate()


        ##params
        elif params != {} and objects is None:
            try: 
                scenario = scenic.scenarioFromFile(self.scenic_file, model="scenic.simulators.metadrive.model",mode2D=True,params=params)
            except InvalidScenarioError:
                print('Invalid Scenario instance returning original program')
                scenario = self.scenario
            try: 
                new_scene, _ = scenario.generate()
                self.curr_params = params
            except RejectionException:
                print(f"Rejection Exception occurred: returning original scene sample")
                scenario = self.scenario
                new_scene, _ = scenario.generate()
        else:
            scenario = self.scenario
            new_scene, _ =  scenario.generate()
        
        self.curr_bytes = scenario.sceneToBytes(new_scene)
        self.curr_full_params = dict(new_scene.params)
        counted = {k: self.curr_full_params.get(k, 0) for k in self.count_params}
        counted = {}
        for x in self.count_params:
            counted[x] = self.curr_full_params.get(x, 0)

        print("counted: ", counted)
        return new_scene