import random
import time

import numpy as np
import rospy

import gym
from gym import error, utils
from gym.spaces import Box
from gym.utils import seeding

from ripl_control.envs.baxter import *

class REWARD(IntEnum):
    SPARSE = 0
    DENSE = 1

class BaxterReacherEnv(gym.Env):
    #TODO: Make this the base reacher env
    """Simple implementation of a Baxter Reacher Env that is meant to work
    with the real robot.
    """

    metadata = {'render.modes': ['human']}

    def __init__(self,
                 sim=False,
                 state_type='ee_position',
                 control = 'joint_positions',
                 reward_type=REWARD.SPARSE,
                 arm="right"):
        self.sim = sim
        self.arm = arm
        self.state_type = state_type
        self.reward_type = reward_type
        self.goal = self._sample_goal()

        if self.sim:
            import pybullet as p
            self._p = p
            p.connect(p.Direct)
        else:
            # initialize ros node
            rospy.init_node("reacher_env")
        self.baxter = Baxter(sim=self.sim,
                             arm=self.arm,
                             control='joint_positions',
                             state_type=state_type)
        # goal threshold
        self.threshold = 1e-2

    def reset(self):
        self.baxter.reset()
        self.goal = self._sample_goal()
        return

    def seed(self):
        # Implement
        return

    def step(self, action):
        self._apply_action(action)
        obs = self._get_obs()
        dist = np.linalg.norm(np.array(self.goal) - np.array(obs))
        near_goal = dist < self.threshold
        if near_goal:
            done = True
        else:
            done = False
        reward = self.compute_reward(dist)
        return obs, reward, done, {}

    def _apply_action(self, action):
        self.baxter.apply_action(action)
        self.baxter.control_rate.sleep()

    def _get_obs(self):
        return self.baxter.get_state()

    def compute_reward(self, dist=None):
        near_goal = dist < self.threshold
        if self.reward_type == REWARD.SPARSE:
            reward = 1 if near_goal else 0
        else:
            reward = 1 / dist
        return reward

    def _sample_goal(self):
        # TODO: change bounds on goal sampling
        x = np.random.uniform(.9, 1., 1)[0]
        y = np.random.uniform(-1, -.55, 1)[0]
        z = np.random.uniform(-.16, .32, 1)[0]
        if self.arm == 'left':
            y *= -1
        return [x, y, z]

    def render(self, mode='human', close=False):
        if self.sim:
            pass
        else:
            pass

    # TODO Change action space and observaton space
    @property
    def action_space(self):
        return Box(-np.inf, np.inf, (self.baxter.get_action_dimension(),))

    @property
    def observation_space(self):
        return Box(-np.inf, np.inf, (self.baxter.get_action_dimension() + len(self.goal),))
