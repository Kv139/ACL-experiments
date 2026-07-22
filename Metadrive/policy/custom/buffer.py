import scenic
import math
import random

from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
import gymnasium as gym
from gymnasium import spaces

from typing import Callable

from scenic.core.errors import InvalidScenarioError
from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario, Scene
from scenic.core.distributions import RejectionException, RandomControlFlowError
from scenic.core.serialization import SerializationError

##scene is composed of object count, type, and global parametetrs
## e.g., car, bedestrian, global weather = rain, etc. ##treat each as one param.
## potentially separate objects and parameters.
##for objects/params not agent behavior

##mvp: category with x feature_nums contains scenes with x objects/global params
# then crossover/mutations

class Scene:
    """one scene reconstruction"""
    def __init__(self, full_params, scene_id, pvl, params=None, scenario=None, s_bytes=None):
        self.scene_id = scene_id
        self.pvl = float(pvl)
        self.full_params = full_params
        self.params = params
        self.scenario = scenario
        self.s_bytes = s_bytes
        self.samples = 0    

    def get_pvl(self):
        return self.pvl
    
    def update(self, new_pvl=None, new_params=None, new_scenario=None, new_bytes=None):
        if new_pvl is not None:
            self.pvl = float(new_pvl)
        if new_params is not None:
            self.params = new_params
        if new_scenario is not None:
            self.scenario = new_scenario
        if new_bytes is not None:
            self.s_bytes = new_bytes

    # def read_scene(self, fall_scenario, scenic_file): ## dpeends on the sceenario so that scenic can compile a scne
    #     if self.params is not None:
    #         try: # arbritrary mutations may not be valid -- ensure that the constructed scenario is
    #             scenario = scenic.scenarioFromFile(
    #                 scenic_file,
    #                 model="scenic.simulators.metadrive.model",
    #                 mode2D=True,
    #                 params=self.params
    #             )
    #         except InvalidScenarioError:
    #             scenario = fall_scenario

    #     elif self.scenario is not None:
    #         scenario = self.scenario
    #     else:
    #         scenario =  fall_scenario
        
    #     scene = scenario.sceneFromBytes(self.s_bytes)
    #     return scene

    def read_scene(self, fall_scenario, scenic_file): 
        if self.scenario is not None:
            scenario = self.scenario
        elif self.params is not None:
            try:
                scenario = scenic.scenarioFromFile(
                    scenic_file,
                    model="scenic.simulators.metadrive.model",
                    mode2D=True,
                    params=self.params,
                )
            except InvalidScenarioError:
                scenario = fall_scenario
        else:
            scenario = fall_scenario

        scene = scenario.sceneFromBytes(self.s_bytes)
        return scene

class Category:
    """sub-buffer, k categories"""
    def __init__(self, capacity, category_id, feature_num, **kwargs):
        # kwargs swallows choices/buckets for signature compatibility, unused here
        self.category_id = category_id
        self.capacity = capacity
        self.scene_list = []         
        self.samples = 0
        self.feature_num = feature_num

        ##for future implementation
        self.pctg = 100
        if (feature_num != 0):
           self.pctg = 100 / feature_num

    def check_full(self):
        l = len(self.scene_list)
        if (l >= self.capacity):
            return True
        return False
    
    def add(self, scene: Scene):
        """add scene to kth buffer/category"""
        if not self.check_full(): 
            self.scene_list.append(scene)
            return None, True
        
        mi = 0
        for i in range(len(self.scene_list)):
            if (self.scene_list[i].pvl < self.scene_list[mi].pvl):
                mi = i

        if (self.scene_list[mi].pvl < scene.pvl):
            out = self.scene_list[mi].scene_id
            self.scene_list[mi] = scene
            return out, True  
        
        return None, False
 

    def update(self, scene_id, new_pvl=None, new_params=None, new_scenario=None, new_bytes=None):
        for x in self.scene_list:
            if x.scene_id == scene_id:
                x.update(new_pvl, new_params, new_scenario, new_bytes)
                return True
        return False
    
    def get_scene(self, scene_id):
        for x in self.scene_list:
            if x.scene_id == scene_id:
                return x
        
        return None
    
    def sample(self): # call twice for crossover
        #potentially add a k argument
        if len(self.scene_list) == 0:
            return None
        
        self.samples += 1
        w = []
        for i in self.scene_list:
            pvl = i.pvl
            if pvl < 1e-9:
                w.append(1e-9)
            else:
                w.append(pvl)
       
        return random.choices(self.scene_list, weights=w, k=1)[0]
    
    
    def mean_pvl(self):
        if len(self.scene_list) == 0:
            return 0.0
        total = 0.0
        for x in self.scene_list:
            total += x.pvl

        ret = total / len(self.scene_list)
        return ret

    def total_scenes(self):
        return len(self.scene_list)
    

class Buffer:
    """full buffer"""
    def __init__(self, k, capacity, category_desc, count_params, choices=None, buckets=None):
        self.k = k
        self.capacity = capacity
        self.categories = {} #dictionary category id to category object
        self.count_params = count_params
        self.choices = choices or {}
        self.buckets = buckets or {}
        for i, feature_num in enumerate(category_desc):
            self.categories[i] = Category(capacity, i, feature_num)
        
        
        self.scene_categories = {} #scene in which category, dict scene_id: cat_id
        
    
    def total_obj(self, full_params):
        total = 0
        for i in self.count_params:
            x = full_params.get(i,0)
            total += x
        return total

    def get_category(self, full_params):
        if full_params is None:
            return 0
        count = self.total_obj(full_params)
        for x in sorted(self.categories):
            if (count <= self.categories[x].feature_num):
                return x

        return max(self.categories)        
    

    def add(self, scene_id, full_params, pvl, params=None, scenario=None, s_bytes=None):
        """add scene to category"""
        category_id = self.get_category(full_params)
        scene = Scene(full_params, scene_id, pvl, params=params, scenario=scenario, s_bytes=s_bytes)

        out, b = self.categories[category_id].add(scene)
        if (b):
            self.scene_categories[scene_id] = category_id
            if out is not None:
                self.scene_categories.pop(out, None) #pop dictionary by key

    def update(self, scene_id, new_pvl=None, new_params=None, new_scenario=None, new_bytes=None):
        category_id = self.scene_categories.get(scene_id)
        if category_id is None:
            return False
        self.categories[category_id].update(scene_id, new_pvl, new_params, new_scenario, new_bytes)
        return True

    def _category_weights(self, filled, timesteps, total):
        """Difficulty-progress weights over `filled` categories.
        Returns None when uniform sampling is more appropriate:
          - only one category to choose from,
          - fewer than two distinct finite feature_nums (no curriculum ordering),
          - or every computed weight collapses to zero.
        """
        if len(filled) <= 1:
            return None
        finite_feats = [c.feature_num for c in filled if math.isfinite(c.feature_num)]
        if len(finite_feats) < 2 or len(set(finite_feats)) < 2:
            return None
        maxf = max(finite_feats)
        progress = timesteps / total if total > 0 else 0.0
        progress = min(max(progress, 0.0), 1.0)
        weights = []
        for c in filled:
            if not math.isfinite(c.feature_num):
                weights.append(1.0)   
                continue
            diff = c.feature_num / maxf
            w = (1 - progress) * (1 - diff) + progress * diff
            weights.append(w if (math.isfinite(w) and w >= 0) else 0.0)
        if not any(w > 0 for w in weights):
            return None
        return weights

    #helper for getting scene for mutation
    def sample_category(self, timesteps, total):
        filled = [c for c in self.categories.values() if c.total_scenes() > 0]
        if len(filled) == 0:
            return None
        weights = self._category_weights(filled, timesteps, total)
        if weights is None:
            return random.choice(filled)
        return random.choices(filled, weights=weights, k=1)[0]
    

    ##select scene for mutation
    def sample_scene(self, agent_steps, total_timesteps):
        category = self.sample_category(agent_steps, total_timesteps)
        if category is None:
            return None
        if category.total_scenes() == 0: 
            return None

        weights = []
        for s in category.scene_list:
            w = s.pvl
            if w < 1e-6:
                w = 1e-6
            weights.append(w)

        ret = random.choices(category.scene_list, weights=weights, k=1)[0]
        ret.samples += 1
        category.samples += 1
        return ret

    ##these simply seelct scenes for crossover

    def crossover_same(self, agent_steps, total_timesteps):
        non_empty = [c for c in self.categories.values() if c.total_scenes() >= 2]
        if not non_empty:
            return None
        weights = self._category_weights(non_empty, agent_steps, total_timesteps)
        if weights is None:
            category = random.choice(non_empty)
        else:
            category = random.choices(non_empty, weights=weights, k=1)[0]

        # PVL-weighted within category (same as sample_scene)
        scene_weights = [max(s.pvl, 1e-6) for s in category.scene_list]
        scene1 = random.choices(category.scene_list, weights=scene_weights, k=1)[0]
        scene2 = random.choices(category.scene_list, weights=scene_weights, k=1)[0]

        scene1.samples += 1
        scene2.samples += 1
        category.samples += 2
        return scene1, scene2

    def crossover_diff(self, timesteps, total):
        
        for i in range(1000):
            n1 = self.sample_category(timesteps, total)
            n2 = self.sample_category(timesteps, total)
            if n1 is None or n2 is None:
                return None
            if (n1 != n2):
                s1 = n1.sample()
                s2 = n2.sample()
                return s1, s2
        return None
    
    def get_scene(self, scene_id):
        category_id = self.scene_categories.get(scene_id)
        if category_id is None:
            return None
        return self.categories[category_id].get_scene(scene_id)

    def total_scenes(self):
        return sum(cat.total_scenes() for cat in self.categories.values())

    def total_pvl(self):
        return sum(s.pvl for cat in self.categories.values() for s in cat.scene_list)
    
    def snapshot(self, episode: int) -> dict:
        row = {"episode": episode, "total_scenes": self.total_scenes()}
        for cid in sorted(self.categories):
            cat = self.categories[cid]
            row[f"cat{cid}_n"] = cat.total_scenes()
            row[f"cat{cid}_pvl_mean"] = cat.mean_pvl()
            row[f"cat{cid}_samples"] = cat.samples
        return row
        # ----- diagnostics -----

    def summary(self, verbose=False):
        """print buffer contents by category"""
        print(f"\n=== Buffer state | total scenes: {self.total_scenes()} ===")
        for cid in sorted(self.categories):
            cat = self.categories[cid]
            n = cat.total_scenes()
            if n == 0:
                print(f"  cat {cid} (feature_num<={cat.feature_num}): empty")
                continue
            pvls = [s.pvl for s in cat.scene_list]
            print(f"  cat {cid} (feature_num<={cat.feature_num}): "
                  f"n={n}/{cat.capacity}, "
                  f"pvl min={min(pvls):.3f} mean={sum(pvls)/n:.3f} max={max(pvls):.3f}, "
                  f"samples_drawn={cat.samples}")
            if verbose:
                for s in cat.scene_list:
                    counts = {p: s.full_params.get(p, 0) for p in self.count_params}
                    print(f"    id={s.scene_id} pvl={s.pvl:.3f} counts={counts}")

    def value_distribution(self):
        """print value distribution per param, across all stored scenes"""
        if not self.choices:
            return
        print("=== Value distribution (all categories) ===")
        for p in self.choices:
            # skip Scenic-sampled params (choices=None) — live Road/Lane objects
            # with weakref hashes that raise when the network is stale
            if self.choices[p] is None:
                continue
            counts = {}
            for cat in self.categories.values():
                for s in cat.scene_list:
                    if s.full_params:
                        v = s.full_params.get(p)
                        counts[v] = counts.get(v, 0) + 1
            if counts:
                print(f"  {p}: {dict(sorted(counts.items(), key=lambda x: (x[0] is None, x[0])))}")

    def bucket_occupancy(self):
        """kept for backward compatibility with gym_w_buffer.py — delegates to value_distribution"""
        self.value_distribution()