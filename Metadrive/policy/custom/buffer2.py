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

    def read_scene(self, fall_scenario, scenic_file):
        if self.params is not None:
            try:
                scenario = scenic.scenarioFromFile(
                    scenic_file,
                    model="scenic.simulators.metadrive.model",
                    mode2D=True,
                    params=self.params
                )
            except InvalidScenarioError:
                scenario = fall_scenario

        elif self.scenario is not None:
            scenario = self.scenario
        else:
            scenario =  fall_scenario

        scene = scenario.sceneFromBytes(self.s_bytes)
        return scene


class Category:
    """sub-buffer, k categories"""
    def __init__(self, capacity, category_id, feature_num, choices, buckets):
        self.category_id = category_id
        self.capacity = capacity
        self.scene_list = []
        self.samples = 0
        self.feature_num = feature_num

        self.choices = {}
        for k, v in choices.items():
            if v is not None:
                self.choices[k] = v

        self.buckets = buckets

        self.pctg = 100
        if (feature_num != 0):
           self.pctg = 100 / feature_num

    def check_full(self):
        l = len(self.scene_list)
        if (l >= self.capacity):
            return True
        return False

    def get_bucket(self, param, x):
        n = self.buckets.get(param, 1) ## number pf buckets
        c = self.choices[param]
    # Defensive: Scenic-corrupted values can slip through. Snap to nearest choice.
        if x not in c:
            if isinstance(x, (int, float)):
                x = min(c, key=lambda v: abs(v - x))
            else:
                x = c[0]
        idx = c.index(x)
        # idx = self.choices[param].index(x)
        bucket_size = math.ceil(len(self.choices[param]) / n)
        return idx // bucket_size

    def get_sub_cap(self, param):
        n = self.buckets.get(param, 1)
        q = math.ceil(self.capacity / n)
        return q

    def value_counts(self, param):
        counts = {}
        for s in self.scene_list:
            if s.full_params:
                x = s.full_params.get(param)
            else:
                x = None
            b = self.get_bucket(param, x)
            counts[b] = counts.get(b, 0) + 1
        return counts

    def p_under_cap(self, scene):
        out = []
        for p in self.choices:
            if self.buckets.get(p, 1) <= 1:
                continue
            x = scene.full_params.get(p)
            v = self.get_bucket(p, x)## dictionary key, value of current thingy
            counts = self.value_counts(p) ##how many senes in buffer are in same bucket
            if counts.get(v, 0) < self.get_sub_cap(p):
                out.append(p)
        return out

    def over_sub_cap_scenes(self):
        cache_counts = {}
        for p in self.choices:
            if self.buckets.get(p, 1) <= 1:
                continue
            cache_counts[p] = self.value_counts(p)

        out = []
        for s in self.scene_list:
            for p in cache_counts:
                if s.full_params:
                    x = s.full_params.get(p)
                else:
                    x = None
                v = self.get_bucket(p, x)
                if cache_counts[p].get(v, 0) >= self.get_sub_cap(p):
                    out.append(s)
                    break
        return out

    def add(self, scene: Scene):
        """add scene to kth buffer/category"""
        if not self.check_full():
            would_violate = False
            for p in self.choices:
                if self.buckets.get(p, 1) <= 1:
                    continue
                x = scene.full_params.get(p) if scene.full_params else None
                b = self.get_bucket(p, x)
                counts = self.value_counts(p)
                if counts.get(b, 0) + 1 > self.get_sub_cap(p):
                    would_violate = True
                    break
            
            if not would_violate:
                self.scene_list.append(scene)
                return None, True

        under = self.p_under_cap(scene)
       
        if len(under) > 0:
            candidates = self.over_sub_cap_scenes()
            if len(candidates) > 0:
                # Try candidates in lowest-PVL order until we find one whose eviction
                # doesn't create new over-caps
                sorted_candidates = sorted(candidates, key=lambda s: s.pvl)
                for victim in sorted_candidates:
                    # Simulate: what would counts look like after swap?
                    new_violation = False
                    for p in self.choices:
                        if self.buckets.get(p, 1) <= 1:
                            continue
                        counts = dict(self.value_counts(p))
                        # remove victim's contribution
                        vx = victim.full_params.get(p) if victim.full_params else None
                        vb = self.get_bucket(p, vx)
                        counts[vb] = counts.get(vb, 0) - 1
                        # add scene's contribution
                        sx = scene.full_params.get(p) if scene.full_params else None
                        sb = self.get_bucket(p, sx)
                        counts[sb] = counts.get(sb, 0) + 1
                        # check
                        if counts[sb] > self.get_sub_cap(p):
                            new_violation = True
                            break
                    if not new_violation:
                        self.scene_list.remove(victim)
                        self.scene_list.append(scene)
                        return victim.scene_id, True
                # no safe victim found → fall through
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

    def sample(self):
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
        total = 0.0
        for x in self.scene_list:
            total += x.pvl

        ret = total / len(self.scene_list)
        return ret

    def total_scenes(self):
        return len(self.scene_list)


class Buffer:
    """full buffer"""
    def __init__(self, k, capacity, category_desc, count_params, choices, buckets):
        self.k = k
        self.capacity = capacity
        self.categories = {}
        self.count_params = count_params
        self.choices = choices
        self.buckets = buckets
        for i, feature_num in enumerate(category_desc):
            self.categories[i] = Category(capacity, i, feature_num, choices, buckets)


        self.scene_categories = {}


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
                self.scene_categories.pop(out, None)

    def update(self, scene_id, new_pvl=None, new_params=None, new_scenario=None, new_bytes=None):
        category_id = self.scene_categories.get(scene_id)
        if category_id is None:
            return False
        self.categories[category_id].update(scene_id, new_pvl, new_params, new_scenario, new_bytes)
        return True

    #helper for getting scene for mutation
    def sample_category(self, timesteps, total):
        filled = []
        for n in self.categories.values():
            if len(n.scene_list) > 0:
                filled.append(n)

        if len(filled) == 0:
            return None

        maxf =  1
        for i in self.categories.values():
            maxf =  max(maxf, i.feature_num)

        progress = timesteps/total
        ##probability of picking category i, higher if feature num is lower and ealiers
        w =[]
        for i in filled:
            diff = i.feature_num / maxf
            ##1 - diff -> if difficulty is easy, more likely to be picked at beginning
            ##1 - progress: if progress is l
            prob = (1 - progress) * (1 - diff) + (progress * diff)
            w.append(prob)

        if sum(w) == 0:
            w = [1.0] * len(filled)

        chosen = random.choices(filled, weights=w, k = 1)[0]
        return chosen


    ##select scene for mutation
    def sample_scene(self, timesteps, total):
        samp_category = self.sample_category(timesteps, total)
        if samp_category is None:
            return None
        samp_scene = samp_category.sample()
        return samp_scene


    ##these simply seelct scenes for crossover

    def crossover_same(self, agent_steps, total_timesteps):
        non_empty = [c for c in self.categories.values() if c.total_scenes() >= 2]
        if not non_empty:
            return None

        progress = agent_steps / total_timesteps
        maxf = max(c.feature_num for c in self.categories.values())
        w = []
        for c in non_empty:
            diff = c.feature_num / maxf
            w.append((1 - progress) * (1 - diff) + progress * diff)
        if sum(w) == 0:
            w = [1.0] * len(non_empty)

        category = random.choices(non_empty, weights=w, k=1)[0]

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
                return s1, s2 ## scene should be in s2.
        return None

    def get_scene(self, scene_id):
        category_id = self.scene_categories.get(scene_id)
        if category_id is None:
            return None
        return self.categories[category_id].get_scene(scene_id)


    def total_scenes(self):
        total = 0
        for c in self.categories.values():
            total += c.total_scenes()
        return total



    def summary(self, verbose=False):
        """Print buffer contents by category."""
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

    def bucket_occupancy(self):
        """Per-category bucket counts. Flags any bucket that exceeds sub_cap."""
        print("\n=== Per-category bucket occupancy ===")
        violations = 0
        for cid in sorted(self.categories):
            cat = self.categories[cid]
            n = cat.total_scenes()
            if n == 0:
                continue
            print(f"\ncat {cid} (feature≤{cat.feature_num}, n={n}/{cat.capacity}):")
            for p in self.choices:
                if self.buckets.get(p, 1) <= 1:
                    continue
                counts = cat.value_counts(p)
                sub_cap = cat.get_sub_cap(p)
                nonzero = {b: c for b, c in sorted(counts.items()) if c > 0}
                over_buckets = [b for b, c in nonzero.items() if c > sub_cap]
                if over_buckets:
                    violations += 1
                    flag = f"  ⚠ OVER (bucket {over_buckets})"
                else:
                    flag = ""
                print(f"  {p:<26} {str(nonzero):<22} sub_cap={sub_cap}{flag}")
        print()
        if violations:
            print(f"⚠ {violations} sub_cap violation(s) — buffer's diversity eviction is broken")
        else:
            print("all sub_cap limits respected")