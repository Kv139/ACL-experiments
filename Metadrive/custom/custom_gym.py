from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
import gymnasium as gym
from gymnasium import spaces
from typing import Callable
from metadrive.envs import MetaDriveEnv


from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario, Scene
from scenic.core.distributions import RejectionException 
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
    verifai_sampler now not an argument added in here, but one specified in the Scenic program
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
                 genetic_flag : bool = True): # empty string means just pure scenic???

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

        self.episode_counter = 0 # id to map instances
        self.episode_plvs = {}
        self.previous_scenes = {}
        self.previous_scenes_params = {}


        self.gae_lambda = 0.95
        self.gamma      = 0.99
        self.pvl_threshold = 0

        self.episode_rewards = []
        self.episode_values  = []

        self.scenic_file = file


    def _make_run_loop(self):
        while True:
            try:
                scene = self.get_scene()
                with self.simulator.simulateStepped(scene, maxSteps=self.max_steps) as simulation:
                    steps_taken = 0
                    # this first block before the while loop is for the first reset call
                    done = lambda: not (simulation.result is None) 
                    truncated = lambda: (steps_taken >= self.max_steps) or simulation.get_truncation()  # TODO handle cases where it is done right on maxsteps
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
        self.rewards = []
        self.values  = []
        if self.loop is None:
            self.loop = self._make_run_loop()
            observation, info = next(self.loop) # not doing self.scene.send(action) just yet
        else:
            observation, info = self.loop.throw(ResetException())
        return observation, info
        
    def step(self, action):
        assert not (self.loop is None), "self.loop is None, have you called reset()?"

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

    def compute_episode_pvl(self):
        """
        Docstring for compute_episode_pvl
        
        :Compute the average postive value loss per episode 
        """
        lastgaelam = 0 
        advantages = [0] * len(self.episode_rewards) # hold the  
        for t in reversed(range(len(self.episode_rewards))):
            if t == len(self.episode_rewards) - 1:
                next_v = 0
                nextnonterminal = 0
            else:
                next_v = self.episode_values[t+1]
                nextnonterminal = 1
            delta = self.episode_rewards[t] + self.gamma * next_v * nextnonterminal - self.episode_values[t]
            advantages[t] = lastgaelam = delta[0] + self.gamma * self.gae_lambda * nextnonterminal * lastgaelam
            advantages[t] = max(advantages[t],0)        

        pvl = np.sum(advantages)/len(advantages)

        if pvl > self.pvl_threshold or self.curr_scene_id < 10:
            self.episode_plvs[self.curr_scene_id] = np.sum(advantages)/len(advantages)
            self.curr_scene_id += 1
    
    def get_scene(self):
        """
        Select next training scene:
            case (1): Not enough scenes have been generated: sample a new scene
            case (2): Enough scenes, and even episode: train an a random new or mutated scene
            case (3): Enough scenes, and odd  episode: train on a old scene or sample a new one
        
        TODO : Add a weighted probability calculation for previously seen scenes
        """
        self.select_best_scenes()
      
        if self.episode_counter < 5:
            self.curr_scene_id = self.episode_counter
            scene = self.generate_scene()
  
        elif self.episode_counter >= 5 and self.episode_counter % 2 == 0: # TODO adjust timing conditions 50/50 exploitation vrs. exploration
            idx1 = random.choice(self.best_scene_ids) #TODO fix size
            choice = random.random()
            if choice < .5: #TODO fix logic -- debugging
                idx2 = random.choice(self.best_scene_ids) #TODO fix size
                if idx1 == idx2:
                    scene = self.read_scene_bytes(idx1)
                    return scene
                else:
                    scene1 = self.read_scene_bytes(idx1)
                    scene2 = self.read_scene_bytes(idx2)
                    scene = self.crossover_scences(scene1=scene1, scene2=scene2)
            else:
                scene = self.mutate_scene(self.read_scene_bytes(idx1))

        else:
            choice = random.random()
            if choice < .5:
                idx = random.choice(self.best_scene_ids)
                self.curr_scene_id = idx
                scene = self.read_scene_bytes(idx)
            else:
                scene = self.generate_scene()
                

        self.episode_counter += 1
        # print(f"checking logs: {self.episode_plvs}")
        return scene


    def crossover_scences(self, scene1, scene2):
        """
        Generate a new program with traits from two differnt programs 
        """
        # This bit is probably unnesecary but I will leave it like this for now
        unmutable_params = ["map", "carla_map", "time_step", "verifaiSamplerType", "render", "use2DMap"]
        mutable_params = [key for key in scene1.params.keys() if key not in unmutable_params]
        
        params = scene1.params
        for key in mutable_params:
            choice = random.random()
            if choice < .5:
                params[key] = scene2.params[key]        

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
        mutable_params = ["select_road","extra_cars" ] 
        conditioned_params = {}
        idx = random.randint(0,len(mutable_params)-1)

        if idx == 2:
            new_val = random.randint(0,6)
            conditioned_params[mutable_params[idx]] = new_val
        else:
            conditioned_params[mutable_params[idx]] = scene.params[mutable_params[idx]]
       
        new_scene = self.generate_scene(params=conditioned_params)
        return new_scene


    def select_best_scenes(self):
        """
        Sort the key-pair matching by value -- then select only the best 100. 
        """
        if self.episode_counter >= 5:
            sorted_pairs = sorted(self.episode_plvs.items(), key=lambda item: item[1], reverse=True)
            total_pairs = min(100, len(sorted_pairs))
            self.best_scene_ids = [idx_value_pairs[0] for idx_value_pairs in sorted_pairs[:total_pairs]] #TODO fix this for modified buffer size
            self.pvl_threshold = np.mean([pair[1] for pair in sorted_pairs[:total_pairs]])

    def read_scene_bytes(self,id):
        """
        Docstring for read_scene
    
        :param scene_bytes: Scenic program written to bytes
        returns: Scene
        """
        if id in self.previous_scenes_params:
            params = self.previous_scenes_params[id]
            scenario = scenic.scenarioFromFile(self.scenic_file,
                                model="scenic.simulators.metadrive.model",
                                mode2D=True,
                                params=params)
        else:
            params = {}
            scenario =  self.scenario
        
        try: 
            if params != {}:
                print("attempting to load mutated scene")
            bytes = self.previous_scenes[id]
            return scenario.sceneFromBytes(bytes)
        
        except (SerializationError, KeyError):
            print(f"SerializationError or KeyError occured returning new Scene")
            print(f"failed id was {id}")
            print(f"previous scenes was {self.previous_scenes}")
            print(f"episode plvs was {self.episode_plvs}")
            """
            Remove broken training scenario or Key from the retained values to prevent reoccurance
            """
            if id in self.episode_plvs:
                del self.episode_plvs[id]
            if id in self.previous_scenes_params:
                print("mutated scenario failed")
                del self.previous_scenes_params[id]
           
            scene, _ = self.scenario.generate()
            return scene


    def generate_scene(self,params={}):
        """
        Generate a new Scenario 
            (1): If no params are passed generates a new scene from the original program
            (2): If custom params are passed compiles a new program with these values and 
                 saves those params so the program can be reconstructed later
        """
        if params is not {}:
            try:
                scenario = scenic.scenarioFromFile(self.scenic_file, model="scenic.simulators.metadrive.model",mode2D=True,params=params)
            except InvalidScenarioError:
                print('Invalid Scearnio instance returning original program')
                scenario = self.scenario

            self.previous_scenes_params[self.curr_scene_id] = params
            try: 
                new_scene, _ = scenario.generate()
            
            except RejectionException:
                print(f"Rejection Exception occurred: returning original scene sample")
                scenario = self.scenario
                new_scene, _ = scenario.generate()
        else:
            scenario = self.scenario
            new_scene, _ =  scenario.generate()
        
        bytes = scenario.sceneToBytes(new_scene)
        self.previous_scenes[self.curr_scene_id] = bytes

        return new_scene

