from scenic.gym import ScenicGymEnv
import scenic
from franka_simulator import WebotsFrankaSimulator

import gymnasium as gym
import numpy as np

from controller import Supervisor # This allows for collecting the Supervisor node which is needed for creating a Webots simulation

from stable_baselines3 import A2C,PPO

from stable_baselines3.common.monitor import Monitor

import time


start = time.time()
supervisor = Supervisor() # Collect the Supervisor node from the simulation
prefix = scenic.__file__[:-22]

simulator = WebotsFrankaSimulator(supervisor) # Create an instance of the WebotsSimulator with the corresponding node

action_space = gym.spaces.Box(low=np.full(shape=(6,),fill_value=-1), high=np.full(shape=(6,), fill_value=1))
observation_space = gym.spaces.Box( 
    low=np.array([-np.inf,-np.inf, -np.inf, -2.8972, -1.7628, -2.8972, -3.0718, -2.8972, -0.0175, -2.8972]),
    high=np.array([np.inf, np.inf,  np.inf,  2.8972,  1.7628,  2.8972,  3.0718,  2.8972,  0.0175,  2.8972]),
    dtype=np.float64                                  
)

                             
max_steps = 10000
episodes  = 100
total_timesteps = max_steps * episodes

scenario = scenic.scenarioFromFile("../../scenarios/panda_goal_reaching.scenic",
                                model="scenic.simulators.webots.model",
                                mode2D=False) # Compile a scenic program 
             
# Create a Gym instance with a the webots simulator and the compiled Scenic program
env = Monitor(ScenicGymEnv(scenario, 
                simulator, 
                render_mode=None, 
                max_steps=max_steps, 
                action_space=action_space,
                observation_space=observation_space,
                feedback_fn=None)) 

env.reset()

# Example usage - this env can be used with libraries such as stable-baselines3

terminated, truncated = False, False
for i in range(10):
    print(f'Starting episode {i}')
    for j in range(100):
        action = action_space.sample()
        observation, reward, terminated, truncated, info = env.step(action)
    env.reset()

