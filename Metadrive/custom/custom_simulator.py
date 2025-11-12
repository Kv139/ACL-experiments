import scenic
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
from scenic.simulators.metadrive.simulator import MetaDriveSimulation
import gymnasium as gym
from gymnasium import spaces
from typing import Callable
import scenic.simulators.metadrive.utils as utils
from metadrive.envs import MetaDriveEnv
from scenic.gym import ScenicGymEnv


class CustomMetaDriveSimulator(DrivingSimulator):
    """Implementation of `Simulator` for MetaDrive."""

    def __init__(self,timestep=0.1,render=True,render3D=False,sumo_map=None,real_time=True):
        super().__init__()
        self.render = render
        self.render3D = render3D if render else False
        self.scenario_number = 0
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.real_time = real_time
        self.scenic_offset, self.sumo_map_boundary = utils.getMapParameters(self.sumo_map)
        if self.render and not self.render3D:
            self.film_size = utils.calculateFilmSize(self.sumo_map_boundary, scaling=5)
        else:
            self.film_size = None

        

    def createSimulation(self, scene, *, timestep, **kwargs):
        self.scenario_number += 1
        return CustomMetaDriveSimulation(
            scene,
            render=self.render,
            render3D=self.render3D,
            scenario_number=self.scenario_number,
            timestep=self.timestep,
            sumo_map=self.sumo_map,
            real_time=self.real_time,
            scenic_offset=self.scenic_offset,
            sumo_map_boundary=self.sumo_map_boundary,
            film_size=self.film_size,
            **kwargs,
        )



class CustomMetaDriveSimulation(MetaDriveSimulation):
    def __init__(
        self,
        scene,
        render,
        render3D,
        scenario_number,
        timestep,
        sumo_map,
        real_time,
        scenic_offset,
        sumo_map_boundary,
        film_size,
        **kwargs,
    ):
        self.scene = scene
        
        super().__init__(scene,render,render3D,scenario_number,timestep,sumo_map,
        real_time,scenic_offset,sumo_map_boundary,film_size,**kwargs,)
        

        def get_obs(self):
            return 0
        
        def get_reward(self):
            return self.scene.objects[0].reward
    

