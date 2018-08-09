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

from baxter_envs.baxter_reacher_env import BaxterReacherEnv

# env = BaxterReacherEnv()
env = gym.make('BaxterReacherEnv-v0')
# env = Monitor(env, "~/",force=True)
env.reset()

def choose_random_action():
    x = np.random.uniform(0, 0.9, 1)[0]
    y = np.random.uniform(-1, 0, 1)[0]
    z = np.random.uniform(.3, .32, 1)[0]
    j1 = np.random.uniform(0, 0.3, 1)
    j2 = np.random.uniform(0, 0.3, 1)
    j3 = np.random.uniform(0, 0.3, 1)
    j4 = np.random.uniform(0, 0.3, 1)
    j5 = np.random.uniform(0, 0.3, 1)
    j6 = np.random.uniform(0, 0.3, 1)
    j7 = np.random.uniform(0, 0.3, 1)
    return [j1, j2, j3, j4, j5, j6, j7]

done = False
while not done:
    # choose random action
    action = choose_random_action()
    # step
    obs, reward, done, _ = env.step(action)
    # print(list(right.endpoint_pose()['position']))
    time.sleep(2)


print(reward)
