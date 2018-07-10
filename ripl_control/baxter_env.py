import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import baxter
import random
import pybullet_data




class BaxterEnv(gym.EnV):
  metadata = {
    'render.modes': ['human', 'rgb_array'],
    'video.frames_per_second': 50
  }


  def __init__(self, action_=1, isEnableSelfCollision=True, renders=False, isDiscrete=False, max_steps=1000):
    self._isDiscrete = isDiscrete
    self._timestep = 1./240.
    self.reset()

  def _step(self, actions):
    pass

  def _reset(self):
    pass

  def _render(self, mode='human', close=False):
    pass
