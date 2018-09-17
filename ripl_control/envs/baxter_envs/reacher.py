import random
import time

import numpy as np
import rospy

import gym
from gym import error, utils
from gym.spaces import Box
from gym.utils import seeding

from ripl_control.envs.baxter import *

class BaxterReacherEnv(gym.Env):
    #TODO: Make this the base reacher env
    """Simple implementation of a Baxter Reacher Env that is meant to work
    with the real robot.
    """

    metadata = {'render.modes': ['human']}

    def __init__(self, sim=False, max_steps=1, arms="right"):
        self.sim = sim
        self.arms = arms
        if self.sim:
            # connect to bullet physics server
            self._p = p
            p.connect(p.Direct)
        else:
            # initialize ros node
            rospy.init_node("reacher_env")
        # create baxter
        self.baxter = Baxter(sim=self.sim, arm=self.arms)
        # reset environment
        # self.reset()
        # set max time steps
        # self.max_steps = max_steps
        # set goal threshold
        self.threshold = 1e-2
        self.reset()
        # return

    def step(self, action):
        # take action
        self.baxter.apply_action(action)
        # sleep
        # TODO: do I need to do this?
        self.baxter.control_rate.sleep()
        # get state (obs)
        state = self.baxter.get_state()
        # determine if done
        dist = np.linalg.norm(np.array(self.goal) - np.array(state))
        near_goal = dist < self.threshold
        # if self.t == self.max_steps - 1 or near_goal:
        if near_goal:
            done = True
        else:
            done = False
        reward = 1 if near_goal else 0
        self.t += 1
        return state, reward, done, {}

    def reset(self):
        # reset baxter
        self.baxter.reset()

        # choose goal
        self.goal = self._sample_goal()
        print("Goal: ", self.goal)
        # keep track of timesteps
        self.t = 0
        return

    def seed(self):
        return

    def render(self, mode='human', close=False):
        if self.sim:
            pass
        pass

    @property
    def action_space(self):
        return Box(-np.inf, np.inf, (self.baxter.get_action_dimension(),))

    @property
    def observation_space(self):
        return Box(-np.inf, np.inf, (self.baxter.get_action_dimension() + len(self.goal),))

    def _sample_goal(self):
        #TODO: reimplement so that the goal is chosen
        # based on which arm is chosen
        x = np.random.uniform(.9, 1., 1)[0]
        y = np.random.uniform(-1, -.55, 1)[0]
        z = np.random.uniform(-.16, .32, 1)[0]
        return [x, y, z]

# class BaxterBaseReacherEnv(gym.Env):
#     def __init__(self, ):
#         # initialize ros node
#         # rospy.init_node("reacher_env")
#
#         # self._isDiscrete = isDiscrete
#         # self._timestep = 1./240.
#         # self.reset()
#
#     def step(self, actions):
#         raise NotImplementedError
#
#     def reset(self):
#         # initialilze baxter
#         raise NotImplementedError
#
#     def render(self, mode='human', close=False):
#         pass
#
#     @property
#     def action_space(self):
#         raise NotImplementedError
#
#     @property
#     def observation_space(self):
#         raise NotImplementedError
