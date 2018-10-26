from __future__ import division
from enum import IntEnum
import os
import time
import numpy as np

import gym
from gym import error
from gym.wrappers.monitor import Monitor
from gym.wrappers.time_limit import TimeLimit

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import gym_baxter
import pybullet as p

real_env = gym.make('BaxterReacherEnv-v0')
sim_env = gym.make('BaxterReacherEnv-v1')

for i in range(50):
    real_obs = real_env.reset()
    sim_obs = sim_env.reset()
    done = False
    total_real_reward = 0.0
    total_sim_reward = 0.0
    while not done:
        action = np.random.randint(0, real_env.action_space.n)
        obs, reward, done, _ = sim_env.step(action)
        obs, reward, done, _ = real_env.step(action)
        total_real_reward += reward
        total_sim_reward += reward
    # print("Episode {} end with Total Reward {}".format(i, total_reward))
