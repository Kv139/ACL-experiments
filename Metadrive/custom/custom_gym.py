from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
import gymnasium as gym
from gymnasium import spaces
from typing import Callable
from metadrive.envs import MetaDriveEnv


from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario, Scene
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
                 scenario : Scenario,
                 simulator : Simulator,
                 render_mode=None, 
                 max_steps = 1000,
                 observation_space : spaces.Dict = spaces.Dict(),
                 action_space : spaces.Dict = spaces.Dict(),
                 record_scenic_sim_results : bool = True,
                 feedback_fn : callable = lambda x: x): # empty string means just pure scenic???

        assert render_mode is None or render_mode in self.metadata["render_modes"]

        self.observation_space = observation_space
        self.action_space = action_space
        self.render_mode = render_mode
        self.max_steps = max_steps - 1 # FIXME, what was this about again?
        self.simulator = simulator
        self.scenario = scenario
        self.simulation_results = []

        self.feedback_result = None
        self.loop = None
        self.record_scenic_sim_results = record_scenic_sim_results
        self.feedback_fn = feedback_fn

        self.episode_counter = 0 # id to map instances
        self.episode_plvs = {}
        self.previous_scenes = {}

        self.gae_lambda = 0.95
        self.gamma      = 0.99

        self.episode_rewards = []
        self.episode_values  = []


    def _make_run_loop(self):
        while True:
            try:
                scene = self.get_scene()
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
            self.episode_plvs[self.episode_counter] = np.sum(advantages)/len(advantages)
    
    def get_scene(self):
        """
        Select next training scene:
            case (1): Not enough scenes have been generated sample a new scene
            case (2): Enough scenes, create a new scene through concatentating two high-performing ones
            case (3): Enough scenes, simply train a a previous high-performing scene
        
        TODO : Add a weighted probability calculation for previously seen scenes
        """
        if self.episode_counter < 10:
            scene, _ = self.scenario.generate(feedback=self.feedback_result)
            # if it is a new scene add it to the buffer
            self.previous_scenes[self.episode_counter] = self.scenario.sceneToBytes(scene)
        elif self.episode_counter > 100 and self.episode_counter % 5 == 0: # TODO adjust timing conditions
            idx1 = random.randint(0,100)
            idx2 = random.randint(0,100)
            if idx1 == idx2:
                return self.previous_scenes[self.best_scene_ids[idx1]]
            else:
                scene1 = self.best_scene_ids[idx1]
                scene2 = self.best_scene_ids[idx2]
                scene = self.mutate_scences(scene1=scene1, scene2=scene2)
        else:
            choice = random.random()
            if choice > .5:
                idx = random.randint(0,len(self.best_scene_ids))
                scene = self.scenario.sceneFromBytes(self.previous_scenes[idx])
    
        self.episode_counter += 1
        print(f"checking logs: {self.episode_plvs}")
        return scene


    def mutate_scences(self, scene1, scene2):
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
                params[mutable_params[key]] = scene2.params[key]
                
        new_scene = Scene(objects=scene1.objects, egoObject=scene1.egoObject,params=params, workspace=scene1.workspace)
            
        return new_scene


    def select_best_scenes(self):
        """
        Sort the key-pair matching by value -- then select only the best 100. 
        """
        sorted_pairs = sorted(self.episode_plvs.items, key=lambda item: item[1], reverse=True)
        self.best_scene_ids = sorted_pairs[:100][0]



