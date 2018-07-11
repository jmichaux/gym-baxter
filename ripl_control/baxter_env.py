import random
import time

import numpy as np
import rospy

import gym
from gym import spaces
from gym.utils import seeding

from baxter_ros import BaxterRos

class BaxterEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50}

    def __init__(self, action_=1, isEnableSelfCollision=True, renders=False, isDiscrete=False, max_steps=1000):
        # initialize ros node
        rospy.init_node("reacher_env", anonymous=False)

        self._isDiscrete = isDiscrete
        self._timestep = 1./240.
        self.reset()

    def _step(self, actions):
        raise NotImplementedError

    def _reset(self):
        # initialilze baxter
        self.baxter = Baxter()
        pass

    def _render(self, mode='human', close=False):
        pass

    @property
    def action_space(self):
        pass

    @property
    def observation_space(self):
        pass
