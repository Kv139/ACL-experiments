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
import csv
import os
import time

from scenic.core.errors import setDebuggingOptions, InvalidScenarioError

from dataclasses import dataclass, asdict, field

@dataclass
class EpisodeRecord:
    episode: int = 0
    op_actual: str = ""
    category_id: int = -1
    #curriculum tracking
    generation_source: str = ""
    params_mutated: list = field(default_factory=list)
    params_crossed: list = field(default_factory=list)
    n_params_differ: int = 0
    #paraeters
    ego_on_intersection: int = 0
    intersection_has_cars: int = 0
    distractor_cars: int = 0
    has_brake_checker: int = 0
    has_cluster: int = 0
    maximum_distractor_speed: float = 0.0
    distance_to_nearest: float = 0.0
    cluster_dist: float = 0.0
    brake_dist: float = 0.0
    intersection_cars: int = 0
    #results
    num_steps: int = 0
    reward_sum: float = 0.0
    pvl: float = 0.0
    #failures  
    failure: str = ""
    termination_reason: str = ""


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
                 start_genetic: int = 25,
                 log_dir: str = "logs",
                 nk_buffer: bool = True,
                 run_name: str = "",
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


        self.nk_buffer = nk_buffer

        ##custom buffer stuff
        self.category_desc = [1, 2, 3, 5, 8, 12, 18]
        self.k = 7

        if self.nk_buffer:
            self.k = 7
            category_desc = self.category_desc
            per_cat_cap = buffer_capacity
        else:
            self.k = 1
            category_desc = [float("inf")]
            per_cat_cap = buffer_capacity 

            
        self.parameters = {
            "select_road": {"choices": None, "group": "ego", "buckets": 1},
            "select_lane": {"choices": None, "group": "ego", "buckets": 1},

            "ego_on_intersection": {"choices": [0, 1], "group": "ego", "buckets": 2},
            "intersection_has_cars": {"choices": [0, 1], "group": "ego", "buckets": 2},

            "distractor_cars": {"choices": [0, 1, 2, 3, 4, 5, 8, 12, 18], "group": "distractors", "buckets": 3},

            "has_brake_checker": {"choices": [0, 1], "group": "brakes", "buckets": 2},
            "intersection_cars": {"choices": [0, 1, 2, 3], "group": "intersect", "buckets": 2},
            "has_cluster": {"choices": [0, 1], "group": "clster", "buckets": 2},
            "maximum_distractor_speed": {"choices": [5, 10, 15, 20], "group": "maxspeed", "buckets": 2},
            "distance_to_nearest": {"choices": [5, 10, 20, 40], "group": "distances", "buckets": 2},
            "cluster_dist": {"choices": [5, 10, 15], "group": "clust_dist", "buckets": 2},
            "brake_dist": {"choices": [10, 20, 35, 55], "group": "brake_dist", "buckets": 2},
        }

        self.count_params = ["distractor_cars"]


        #self.count_params = ["extra_cars", "trees", "buildings"]

        choices = {}
        buckets = {}
        for k, v in self.parameters.items():
            choices[k] = v["choices"]
            buckets[k] = v["buckets"]
        self.buffer = Buffer(k=self.k, count_params=self.count_params, capacity=per_cat_cap, category_desc=category_desc, choices=choices, buckets=buckets)
        self.start_genetic = start_genetic
        self.total_timesteps = total_timesteps
        self.agent_steps = 0 ##agent steps ACROSS ALL EPISODES, fr tottal_tiemsteps ACROSS ALL EPISODES

        self.episode_counter = 0

        self.curr_full_params=None
        self.curr_bytes = None ##prevoius scenes in bytes
        self.curr_params = None ##params for scenarios
        self.curr_scenario = None ##scenario objects
        self._scenario_cache_key = None  # last compiled params key
        self._scenario_cache = None      # last compiled scenario

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

        self.gen_source = "fresh"
        self.last_mutated_params = []
        self.last_crossed_params = []
        self.last_parent_params = None
        self.last_failure = ""
        self.termination_reason = ""
        self.last_info = {}    
        self.run_name = run_name

        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, f"episodes_{run_name}_{timestamp}.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = None

    def log_episode(self, record):
        row = asdict(record)
        row["params_mutated"] = "|".join(row["params_mutated"])
        row["params_crossed"] = "|".join(row["params_crossed"])
        if self.csv_writer is None:
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=list(row.keys()))
            self.csv_writer.writeheader()
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def write_record(self, pvl):
        if not self.termination_reason:
            if len(self.episode_rewards) >= self.max_steps:
                self.termination_reason = "max_step"  
            else:
                self.termination_reason = "unknown"
        p = self.curr_full_params or {}
        n_differ = 0
        if self.last_parent_params is not None:
            n_differ = sum(
                1 for k in self.parameters
                if k in self.last_parent_params and k in p
                and self.last_parent_params[k] != p[k]
            )

        record = EpisodeRecord(
            episode=self.episode_counter - 1,
            op_actual=self.gen_source,
            category_id=self.buffer.get_category(p),
            generation_source=self.gen_source,
            params_mutated=list(self.last_mutated_params),
            params_crossed=list(self.last_crossed_params),
            n_params_differ=n_differ,
            ego_on_intersection=int(p.get("ego_on_intersection", 0)),
            intersection_has_cars=int(p.get("intersection_has_cars", 0)),
            distractor_cars=int(p.get("distractor_cars", 0)),
            has_brake_checker=int(p.get("has_brake_checker", 0)),
            has_cluster=int(p.get("has_cluster", 0)),
            maximum_distractor_speed=float(p.get("maximum_distractor_speed", 0.0)),
            distance_to_nearest=float(p.get("distance_to_nearest", 0.0)),
            cluster_dist=float(p.get("cluster_dist", 0.0)),
            brake_dist=float(p.get("brake_dist", 0.0)),
            intersection_cars=int(p.get("intersection_cars", 0)),
            num_steps=len(self.episode_rewards),
            reward_sum=float(sum(self.episode_rewards)) if self.episode_rewards else 0.0,
            pvl=float(pvl),
            failure=self.last_failure,
            termination_reason=self.termination_reason,
        )
        self.log_episode(record)
        self.last_failure = ""

    def get_term(self, info, simulation, steps_taken):
        if simulation.result is not None:
            return "scenic_monitor"
        info = info or {}
        for i in ("arrive_dest", "crash_vehicle", "crash_object", "crash_sidewalk","out_of_road", "max_step"):
            if info.get(i):
                return i
        if steps_taken >= self.max_steps:
            return "max_step"
        return "scenic_terminate"

    def _make_run_loop(self):
        while True:
            try:
                self.termination_reason = ""
                if self.genetic_flag:           
                    scene = self.get_scene()
                else:
                    # scene, _ = self.scenario.generate(feedback=self.feedback_result)
                    # self.curr_full_params = dict(scene.params) if scene.params else {}
                    self.replay = False
                    self.gen_source = "fresh"
                    self.last_mutated_params = []
                    self.last_crossed_params = []
                    self.last_parent_params = None
                    scene = self.generate_fresh()
                    self.info['generations'] += 1
                #print(f"[episode {self.episode_counter+1}] params: {self.curr_full_params}")
                # if self.episode_counter > 0 and self.episode_counter % 50 == 0:
                #     self.buffer.summary(verbose=False)

                try:
                    with self.simulator.simulateStepped(scene, maxSteps=self.max_steps) as simulation:
                        steps_taken = 0
                        self.episode_counter += 1

                        #print(f'{self.episode_counter}')
        
                        done = lambda: (simulation.result is not None) or simulation.is_done()
                        truncated = lambda: (steps_taken >= self.max_steps) or simulation.get_truncation()
                        observation = simulation.get_obs()
                        info = simulation.get_info() 
                        actions = yield observation, info
                        simulation.actions = actions # TODO add action dict to simulation interfaces

                        while not done():
                            simulation.advance()
                            steps_taken += 1
                            observation = simulation.get_obs()
                            info = simulation.get_info()
                            self.last_info = info if isinstance(info, dict) else {}
                            reward = simulation.get_reward()
                            if done():
                                self.termination_reason = self.get_term(info, simulation, steps_taken)
                                self.last_step_truncated = (
                                    truncated()
                                    and (simulation.result is None)
                                    and self.termination_reason == "max_step"
                                )
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
                    raise
                    
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
        if hasattr(self, "csv_file") and not self.csv_file.closed:
            self.csv_file.close()
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
        pvl = 0.0
        if len(self.episode_rewards)>= 1 and len(self.episode_values) >= 1:
            lastgaelam = 0 
            advantages = [0] * len(self.episode_rewards) # hold the  
            for t in reversed(range(len(self.episode_rewards))):
                if t == len(self.episode_rewards) - 1:
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
                self.write_record(float(pvl))
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

        self.write_record(float(pvl))
    
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
    
    def normalize_params(self, p):
        p = dict(p)
        if p.get("distractor_cars", 0) == 0:
            p["has_cluster"] = 0
            p["intersection_cars"] = 0

        if p.get("ego_on_intersection", 0) == 0:
            p["intersection_has_cars"] = 0

        d = p.get("distractor_cars", 0)
        ic = p.get("intersection_cars", 0)
        if ic > d:
            p["intersection_cars"] = d

        if p.get("has_brake_checker", 0) == 1 and p.get("distractor_cars", 0) > 0:
            dn = p.get("distance_to_nearest", 5)
            bd = p.get("brake_dist", 25)
            margin = 5
            if bd <= dn + margin:
                bd_choices = self.parameters["brake_dist"]["choices"]
                valid = [x for x in bd_choices if x > dn + margin]
                p["brake_dist"] = min(valid) if valid else max(bd_choices)

        return p
    
    def generate_fresh(self):
        p = {}
        for key, value in self.parameters.items():
            if value["choices"] is not None: 
                p[key] = random.choice(value["choices"])
        p = self.normalize_params(p)
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
        self.gen_source = "fresh"
        self.last_mutated_params = []
        self.last_crossed_params = []
        self.last_parent_params = None
        if self.episode_counter % 10 == 0:
            #print(f"Counts: {self.info}")
            self.buffer.summary(verbose=False)
            self.buffer.bucket_occupancy()

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
                    # read_scene_bytes may return None if both replay and fallback failed
                    if s1 is None or s2 is None:
                        scene = self.generate_fresh()
                        self.info['generations'] += 1
                    else:
                        self.gen_source = "crossover"
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
                    self.gen_source = "mutation"
                    scene = self.mutate_scene(scene=s1)
                    self.info['mutations'] += 1
                
            else:
                ret = self.buffer.sample_scene(self.agent_steps, self.total_timesteps)
                if ret is None:
                    scene = self.generate_fresh()
                    self.info['generations'] += 1
                else:
                    self.replay = True
                    self.gen_source = "replay"
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
        from_p2 = []
        for i, keys in self.groups().items():
            c = random.choice([scene1, scene2])
            for k in keys:
                if k in c.params:
                    params[k] = c.params[k]
                    if c is scene2:
                        from_p2.append(k)
        params = self.normalize_params(params)
        self.last_crossed_params = from_p2
        self.last_parent_params = dict(scene1.params) if scene1.params else {}
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
        self.last_mutated_params = list(groups[target])
        self.last_parent_params = dict(scene.params) if scene.params else {}
        params = {}
        for name, keys in groups.items():
            if name == target:
                continue  
            for k in keys:
                if k in scene.params:
                    params[k] = scene.params[k]

        for k in groups[target]:
            if self.parameters[k]["choices"] is not None:
                params[k] = random.choice(self.parameters[k]["choices"])
        params = self.normalize_params(params)
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
        #print(f"[read_scene_bytes] id={scene_id} took_path={'scenario' if scene.scenario is not None else 'params' if scene.params is not None else 'fall'}")
        
        if (scene.scenario is not None): 
            fail = "scenario_failures"
        elif scene.params is not None:
            fail = "param_failures"
        else:
            fail = "generation_failures"
        
        try:
            return scene.read_scene(self.scenario, self.scenic_file)
        except (SerializationError, KeyError, RejectionException) as e: ##getting typerror issue
            #print(f"failed id was {scene_id}, scenario was genetic {fail}, error type was {e}")
            self.info[fail] += 1
            scene, _ = self.scenario.generate()
            self.replay = False
            #return None
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
                #print(f"Control flow error with objects {objects}")
                # for i in range(len(scene.objects)):
                #     if i in objects:
                #         print(scene.objects[i])
                print("")

                scenario = self.scenario
                new_scene, _ = scenario.generate()
                self.curr_scenario = scenario  

        ##params
        elif params != {} and objects is None:
            try:
                key = tuple(sorted(params.items()))
                if self._scenario_cache is not None and key == self._scenario_cache_key:
                    scenario = self._scenario_cache
                else:
                    scenario = scenic.scenarioFromFile(self.scenic_file, model="scenic.simulators.metadrive.model",mode2D=True,params=params)
                    self._scenario_cache_key = key
                    self._scenario_cache = scenario
            except InvalidScenarioError:
                print('Invalid Scenario instance returning original program')
                scenario = self.scenario
            try: 
                new_scene, _ = scenario.generate()
                self.curr_params = params
                self.curr_scenario = scenario 
            except RejectionException:
                self.last_failure = "rejection_fallback"   
                scenario = self.scenario
                new_scene, _ = scenario.generate()
                self.curr_scenario = scenario   
                self.curr_params = None 
        else:
            scenario = self.scenario
            new_scene, _ =  scenario.generate()
            self.curr_scenario = scenario  
            
        self.curr_bytes = scenario.sceneToBytes(new_scene)
        self.curr_full_params = dict(new_scene.params) if new_scene.params else {}
        counted = {}
        for p in self.count_params:
            counted[p] = self.curr_full_params.get(p, 0)
        # print("counted: ", counted)
        
        # print(f"[episode {self.episode_counter+1}] PASSED: {params}")
        # print(f"[episode {self.episode_counter+1}] SCENE : {self.curr_full_params}")
        return new_scene