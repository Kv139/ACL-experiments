from scenic.gym import ScenicGymEnv
import scenic
from custom.custom_simulator import CustomMetaDriveSimulation, CustomMetaDriveSimulator 
from custom.custom_gym import CustomMetaDriveEnv

from scenic.simulators.metadrive import MetaDriveSimulator
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
                             
max_steps = 100
episodes  = 100
total_timesteps = max_steps * episodes

scenario1 = scenic.scenarioFromFile("./scenarios/driver.scenic",
                                model="scenic.simulators.metadrive.model",
                                mode2D=True)

scenario2 = scenic.scenarioFromFile("./scenarios/driver.scenic",
                                model="scenic.simulators.metadrive.model",
                                mode2D=True)
             

env = CustomMetaDriveEnv(
                scenarios=[scenario1,scenario2], 
                simulator=CustomMetaDriveSimulator(sumo_map="./CARLA/Town01.net.xml", max_steps=max_steps),
                observation_space=observation_space,
                action_space=action_space) # max_step is max step for an episode - Create an enviroment instance

""""
Testing something
"""
import torch
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

print(f"displyaing device: {device}")

env.reset()
terminated, truncated = False, False
for episode in range(10):
    for i in range(100):
        print(f"truncated: {truncated}")
        if terminated or truncated:
            print("early break")
            break
        else:
            observation, reward, terminated, truncated, info = env.step([0,1])
            print(info)
    env.reset()
        

    # print(reward)

