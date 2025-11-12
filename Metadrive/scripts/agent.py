from scenic.gym import ScenicGymEnv
import scenic
from custom.custom_simulator import CustomMetaDriveSimulation, CustomMetaDriveSimulator 
from custom.custom_gym import CustomMetaDriveEnv
#from custom_env import MetaDriveEnv

import gymnasium as gym
import numpy as np

# from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy

from metadrive.component.sensors.semantic_camera import SemanticCamera

import matplotlib.pyplot as plt
import time
import gc

start = time.time()


action_space = gym.spaces.Box(low=np.array([-1,-1]), high=np.array([1,1]), shape=(2,), dtype=np.float32)  # Defines the possible actions of the agent

observation_space = gym.spaces.Dict({
    "velocity":  gym.spaces.Discrete(16),
    "sensor": gym.spaces.Box(low=np.array([0,0,0,0,0,0,0]), high=np.array([1,1,1,1,1,1,1]),shape=(7,),dtype=np.float64), # defines the range of observations of the agent
    "position": gym.spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), shape=(2,),dtype=np.float64),
    "rotation": gym.spaces.Box(low=np.array([-1,-1,-1,-1]), high=np.array([1,1,1,1]), shape=(4,), dtype=np.float64)
})
                             
max_steps = 10000
episodes  = 100
total_timesteps = max_steps * episodes

scenario = scenic.scenarioFromFile("./scenarios/driver.scenic",
                                model="scenic.simulators.metadrive.model",
                                mode2D=True)
             
meta_config = dict(
    sensors={"sementic_camera": [SemanticCamera, (16,16)]},
    vehicle_config={"image_source": "sementic_camera"},
    stack_size=3,
)

env = CustomMetaDriveEnv(
                scenario=scenario, 
                simulator=CustomMetaDriveSimulator(sumo_map="./CARLA/Town01.net.xml"),
                max_steps=max_steps,
                observation_space=observation_space,
                action_space=action_space) # max_step is max step for an episode - Create an enviroment instance



# print(dir(env))
env.reset()
for i in range(4):
    things = env.step([0,0])
    for r in things:
        print(r)
