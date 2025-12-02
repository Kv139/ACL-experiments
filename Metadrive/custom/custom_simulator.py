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
from scenic.domains.driving.simulators import DrivingSimulation
from metadrive.component.sensors.rgb_camera import RGBCamera
from metadrive.component.sensors.semantic_camera import SemanticCamera
from metadrive.component.sensors.lidar import Lidar
import numpy as np
"""Simulator interface for MetaDrive."""

try:
    from metadrive.component.traffic_participants.pedestrian import Pedestrian
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
except ImportError as e:
    raise ModuleNotFoundError(
        "Metadrive is required. Please install the 'metadrive-simulator' package (and sumolib) or use scenic[metadrive]."
    ) from e

import logging
import sys
import time

from scenic.core.simulators import InvalidScenarioError, SimulationCreationError
from scenic.domains.driving.actions import *
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
)
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
import scenic.simulators.metadrive.utils as utils


class CustomMetaDriveSimulator(DrivingSimulator):
    """Implementation of `Simulator` for MetaDrive."""

    def __init__(self,timestep=0.1,render=True,render3D=False,sumo_map=None,real_time=True,max_steps=1000):
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
        self.max_steps = max_steps

        

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
            max_steps=self.max_steps,
            **kwargs,
        )



class CustomMetaDriveSimulation(DrivingSimulation):
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
        max_steps,
        **kwargs,
    ):
        if len(scene.objects) == 0:
            raise InvalidScenarioError(
                "Metadrive requires you to define at least one Scenic object."
            )
        if not scene.objects[0].isCar:
            raise InvalidScenarioError(
                "The first object must be a car to serve as the ego vehicle in Metadrive."
            )

        self.render = render
        self.render3D = render3D
        self.scenario_number = scenario_number
        self.defined_ego = False
        self.client = None
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.real_time = False
        self.scenic_offset = scenic_offset
        self.sumo_map_boundary = sumo_map_boundary
        self.film_size = film_size
        self.actions = [0,0]
        self.observation = np.zeros(shape=(84,64,3))
        self.early_terminate = False
        self.max_steps = max_steps
        self.steps_taken = 0
        self.rewards = []
        super().__init__(scene, timestep=timestep, **kwargs)

    def createObjectInSimulator(self, obj):
        """
        Create an object in the MetaDrive simulator.

        If it's the first object, it initializes the client and sets it up for the ego car.
        For additional cars and pedestrians, it spawns objects using the provided position and heading.
        """
        converted_position = utils.scenicToMetaDrivePosition(
            obj.position, self.scenic_offset
        )
        converted_heading = utils.scenicToMetaDriveHeading(obj.heading)

        if not self.defined_ego:
            decision_repeat = math.ceil(self.timestep / 0.02)
            physics_world_step_size = self.timestep / decision_repeat

            sensor_size = (84,64)

            # Initialize the simulator with ego vehicle
            self.client = utils.DriveEnv(
                dict(
                    decision_repeat=decision_repeat,
                    physics_world_step_size=physics_world_step_size,
                    use_render=self.render3D,
                    vehicle_config={
                        "spawn_position_heading": [
                            converted_position,
                            converted_heading,
                        ],
                        "image_source":"semnatic_camera",  
                    },
                    use_mesh_terrain=self.render3D,
                    log_level=logging.CRITICAL,
                    sensors={"semantic_camera": (SemanticCamera, *sensor_size)},
                    stack_size=1,
                    image_observation=True,
                )
            )
            self.client.config["sumo_map"] = self.sumo_map
            self.client.reset()

            # Assign the MetaDrive actor to the ego
            metadrive_objects = self.client.engine.get_objects()
            obj.metaDriveActor = list(metadrive_objects.values())[0]
            self.defined_ego = True
            return

        # For additional cars
        if obj.isVehicle:
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                DefaultVehicle,
                vehicle_config=dict(),
                position=converted_position,
                heading=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor
            return

        # For pedestrians
        if obj.isPedestrian:
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                Pedestrian,
                position=converted_position,
                heading_theta=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor
            return

        # If the object type is unsupported, raise an error
        raise SimulationCreationError(
            f"Unsupported object type: {type(obj)} for object {obj}."
        )

    def executeActions(self, allActions):
        """Execute actions for all vehicles in the simulation."""
        super().executeActions(allActions)

        # Apply control updates to vehicles and pedestrians
        for obj in self.scene.objects[1:]:  # Skip ego vehicle (it is handled separately)
            if obj.isVehicle:
                action = obj._collect_action()
                obj.metaDriveActor.before_step(action)
                obj._reset_control()
            else:
                # For Pedestrians
                if obj._walking_direction is None:
                    obj._walking_direction = utils.scenicToMetaDriveHeading(obj.heading)
                if obj._walking_speed is None:
                    obj._walking_speed = obj.speed
                direction = [
                    math.cos(obj._walking_direction),
                    math.sin(obj._walking_direction),
                ]
                obj.metaDriveActor.set_velocity(direction, obj._walking_speed)

    def step(self):
        start_time = time.monotonic()

        # Special handling for the ego vehicle
        ego_obj = self.scene.objects[0]
        # action = ego_obj._collect_action()
        self.client.step([self.actions[0], self.actions[1]])  # Apply action in the simulator
        ego_obj._reset_control()
        # Render the scene in 2D if needed
        if self.render and not self.render3D:
            self.client.render(
                mode="topdown", semantic_map=True, film_size=self.film_size, scaling=5
            )

        # If real-time synchronization is enabled, sleep to maintain real-time pace
        if self.real_time:
            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            if elapsed_time < self.timestep:
                time.sleep(self.timestep - elapsed_time)
        self.steps_taken += 1

    def destroy(self):

        print(np.mean(self.rewards))

        if self.client and self.client.engine:
            object_ids = list(self.client.engine._spawned_objects.keys())
            if object_ids:
                self.client.engine.agent_manager.clear_objects(object_ids)
            self.client.close()

        super().destroy()

    def getProperties(self, obj, properties):
        metaDriveActor = obj.metaDriveActor
        position = utils.metadriveToScenicPosition(
            metaDriveActor.position, self.scenic_offset
        )
        velocity = Vector(*metaDriveActor.velocity, 0)
        speed = metaDriveActor.speed
        md_ang_vel = metaDriveActor.body.getAngularVelocity()
        angularVelocity = Vector(*md_ang_vel)
        angularSpeed = math.hypot(*md_ang_vel)
        converted_heading = utils.metaDriveToScenicHeading(metaDriveActor.heading_theta)
        yaw, pitch, roll = obj.parentOrientation.globalToLocalAngles(
            converted_heading, 0, 0
        )
        elevation = 0

        values = dict(
            position=position,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            elevation=elevation,
        )

        return values

    def getLaneFollowingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.13, K_D=0.3, K_I=0.05, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getTurningControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.2, K_I=0.2, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.4, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getLaneChangingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.2, K_I=0.02, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.1, K_D=0.3, K_I=0.0, dt=dt)
        return lon_controller, lat_controller


    def get_obs(self):

        # print(dir(self.client.engine))
        # print(dir(self.client.vehicle))

        sem_camera =  self.client.engine.get_sensor("semantic_camera")

        obs = np.array(sem_camera.perceive())

        # print(f'Checking roadDeviation for ego: {self.scene.objects[0].roadDeviation}')


        # print(f"shape was {np.array(obs.shape)}")
        # print(f" observation was {obs}")
        # lidar = self.client.engine.get_sensor("lidar")
        # distances, objects = lidar.perceive(
        #     self.client.vehicle,
        #     self.client.engine.physics_world.dynamic_world,
        #     num_lasers=10,
        #     distance=3,
        # )

        # state = self.client.vehicle.get_state()
        # print(state)

        return obs
    
    def get_truncation(self):
        return self.early_terminate

    
    def get_reward(self):
        """
        Return accumulated reward which is computed in the scenic program
        """
        state = self.client.vehicle.get_state()

        keys = ['crash_object', 'crash_vehicle','crash_building', 'crash_sidewalk']
        if np.any([state[key] for key in keys]):
            self.early_terminate = True
            print("crash")
            return -10
        elif self.scene.objects[0]._lane is None and self.scene.objects[0]._intersection is None:
            print("out of lane")
            self.early_terminate = True
            return -10 # Ego is no longer on the map
        elif self.max_steps == self.steps_taken-1:
            print("finished episode")
            return 5 # finish the episode without leaving the road
        else:
            reward= self.scene.objects[0].reward
        self.rewards.append(reward)
        return reward
    
    def get_info(self):
        """
        Check for collisions 
        """
        state = self.client.vehicle.get_state()

        keys = ['crash_object', 'crash_vehicle','crash_building', 'crash_sidewalk']
        info = {}
        info['crash'] = np.any([state[key] for key in keys])
        info['cte'] = self.scene.objects[0].cte
        info["road_deviation"] = self.scene.objects[0].roadDeviation
        return info
    
    
    def get_feedback(self):
        return 0

