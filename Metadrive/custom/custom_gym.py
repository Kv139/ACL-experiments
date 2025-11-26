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
                 scenarios,
                 simulator : Simulator,
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
        self.scenario = scenarios[0]
        self.mutable_scenario = scenarios[1]
        self.simulation_results = []

        self.genetic_flag = genetic_flag

        self.feedback_result = None
        self.loop = None
        self.record_scenic_sim_results = record_scenic_sim_results
        self.feedback_fn = feedback_fn

        self.episode_counter = 0 # id to map instances
        self.episode_plvs = {}
        self.previous_scenes = {}

        self.gae_lambda = 0.95
        self.gamma      = 0.99
        self.pvl_threshold = 0

        self.episode_rewards = []
        self.episode_values  = []


    def _make_run_loop(self):
        while True:
            try:
                if self.genetic_flag:
                    scene = self.get_scene()
                else:
                    scene = self.scenario.generate()
                with self.simulator.simulateStepped(scene, maxSteps=self.max_steps) as simulation:
                    steps_taken = 0
                    # this first block before the while loop is for the first reset call
                    done = lambda: not (simulation.result is None) 
                    truncated = lambda: (steps_taken >= self.max_steps)or simulation.get_truncation()  # TODO handle cases where it is done right on maxsteps
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

        if pvl > self.pvl_threshold:
            self.episode_plvs[self.curr_scene_id] = np.sum(advantages)/len(advantages)
    
    def get_scene(self):
        """
        Select next training scene:
            case (1): Not enough scenes have been generated sample a new scene
            case (2): Enough scenes, create a new scene through concatentating two high-performing ones
            case (3): Enough scenes, simply train a a previous high-performing scene
        
        TODO : Add a weighted probability calculation for previously seen scenes
        """
        if self.episode_counter < 5:
            self.curr_scene_id = self.episode_counter
            scene, _ = self.scenario.generate(feedback=self.feedback_result)
            # if it is a new scene add it to the buffer
            self.previous_scenes[self.episode_counter] = self.scenario.sceneToBytes(scene)
        
        elif self.episode_counter >= 5 and self.episode_counter % 5 == 0: # TODO adjust timing conditions
            self.select_best_scenes()
            self.curr_scene_id = self.episode_counter
            idx1 = random.randint(0,len(self.best_scene_ids)-1) #TODO fix size
            print(f"Idx1 : {idx1} with total scenes so far: {len(self.best_scene_ids)}")
            choice = random.random()
            if choice < .5:
                idx2 = random.randint(0,len(self.best_scene_ids)-1) #TODO fix size
                if idx1 == idx2:
                    print(f"ids: {self.best_scene_ids[idx1]}")
                    print(f"scene_bytes {self.previous_scenes[self.best_scene_ids[idx1]]}")
                    scene_bytes = self.previous_scenes[self.best_scene_ids[idx1]]
                    scene = self.read_scene_bytes(scene_bytes)
                    return scene
                else:
                    print(f"ids: {self.best_scene_ids}", {idx1}, {idx2})
                    print(f"scene_bytes {self.previous_scenes[self.best_scene_ids[idx1]]}")
                    scene1 = self.read_scene_bytes(self.previous_scenes[self.best_scene_ids[idx1]])
                    scene2 = self.read_scene_bytes(self.previous_scenes[self.best_scene_ids[idx2]])
                    scene = self.crossover_scences(scene1=scene1, scene2=scene2)
                    print("successfully cross-overed scenes")
            else:
                print("entering mutations")
                scene_bytes = self.previous_scenes[self.previous_scenes[self.best_scene_ids[idx1]]]
                scene = self.mutate_scene(self.read_scene_bytes(scene_bytes))
                print("succesfully mutated scene")

        else:
            self.select_best_scenes()
            choice = random.random()
            idx = random.randint(0,len(self.best_scene_ids))
            scene = self.read_scene_bytes(self.previous_scenes[idx])
            self.curr_scene_id = idx
            print("successfully replayed scene")

        
    
        self.episode_counter += 1
        print(f"checking logs: {self.episode_plvs}")
        return scene


    def crossover_scences(self, scene1, scene2):
        """
        Generate a new program with traits from two differnt programs 
        """
        # This bit is probably unnesecary but I will leave it like this for now
        unmutable_params = ["map", "carla_map", "time_step", "verifaiSamplerType", "render", "use2DMap"]
        mutable_params = [key for key in scene1.params.keys() if key not in unmutable_params]

        # try conditionTo to reset the values of the scene parameters
        # Will need to loop through each object and param and reset the values
        # by passing self back

        # or try using two scenarios and using one which is a "throw-away" in order to preserve 
        # the haltons

        # consider applying the genetic algorithm via Verify to compare against
        # could also apply it as a verifAI sampler


        params = scene1.params
        for key in mutable_params:
            choice = random.random()
            if choice < .5:
                params[mutable_params[key]] = scene2.params[key]
                
        self.mutable_scenario.conditionOn(scene=scene1.objects,params=params)
        new_scene = self.mutable_senario.generate()    

        return new_scene
    
    def mutate_scene(self,scene):
        """
        Docstring for mutate_scene

        :param scene: Sampled scenic program instance
        Takes a scenic program and randomly chooses certain parameter values
            then condidtions the distribution to them and resamples
            If no valid sample is found returns the original program
        """
        unmutable_params = ["map", "carla_map", "time_step", "verifaiSamplerType", "render", "use2DMap"]
        mutable_params = [key for key in scene.params.keys() if key not in unmutable_params]
        
        conditioned_params = {}
        for key in mutable_params:
            choice = random.random()
            if choice < .5:
                conditioned_params[key] = scene.params[key]
       
        self.mutable_scenario.conditionOn(scene=scene,params=conditioned_params)
        
        try: 
            new_scene = self.mutable_scenario.generate()
        except RejectionException:
            print(f"Rejection Exception occurred: returning original scene")
            new_scene = scene

        self.reset_scenario()
        return new_scene


    def select_best_scenes(self):
        """
        Sort the key-pair matching by value -- then select only the best 100. 
        """
        sorted_pairs = sorted(self.episode_plvs.items(), key=lambda item: item[1], reverse=True)
        self.best_scene_ids = sorted_pairs[:5][0] #TODO fix this for modified buffer size
        self.pvl_threshold = np.mean(sorted_pairs[:100][1])


    def reset_scenario(self):
        """
        Docstring for reset_scenario
    
        :param scenario: Compiled scenic program
            reset the sampling distribution of the program after condidtioning
        """
        for obj in self.mutable_scenario.objects:
            obj.conditionTo(obj)
        for param in self.mutable_scenario.params.values():
            param.conditionTo(obj)


    def read_scene_bytes(self,scene_bytes):
        """
        Docstring for read_scene
    
        :param scene_bytes: Scenic program written to bytes
        returns: Scene
        """
        try: 
            return self.scenario.sceneFromBytes(scene_bytes)
        except SerializationError:
            print(f"SerializationError occured returning new scene")
            return self.scenario.generate()




