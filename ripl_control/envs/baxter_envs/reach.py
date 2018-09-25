import random
import time

import numpy as np
import rospy

import gym
from gym import error, utils
from gym.spaces import Box
from gym.utils import seeding

from ripl_control.envs.baxter import *
from robot_base_env import RobotBaseEnv

class REWARD(IntEnum):
    SPARSE = 0
    DENSE = 1

class BaxterReacherEnv(robot_base_envself.RobotBaseEnv):
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
        return self._get_obs()

    def _get_obs(self):
        ee_pose = self.baxter.get_ee_pose()
        joint_angles = self.baxter.get_joint_angles()
        achieved_goal = ee_pose[:3]
        obs = np.array(ee_pose + joint_angles)
        {
            'observation': np.array(obs),
            'achieved_goal': np.array(achieved_goal),
            'desired_goal': self.goal,
        }
        return obs

    def _apply_action(self, action):
        self.baxter.apply_action(action)
        if not self.sim:
            self.baxter.control_rate.sleep()

    def compute_reward(self, achieved_goal, desired_goal, info):
        dist = self._calc_distance(achieved_goal, desired_goal)
        if self.reward_type  == REWARD.SPARSE:
            return -(dist > self.threshold).astype(np.float32)
        else:
            return -dist

    def _sample_goal(self):
        # TODO: change bounds on goal sampling
        x = np.random.uniform(.5, 1., 1)[0]
        y = np.random.uniform(-1, 0, 1)[0]
        z = np.random.uniform(-.3, .7, 1)[0]
        if self.arm == 'left':
            y *= -1
        return [x, y, z]

    def _calc_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(a,b, axis=-1)

    def _is_success(self, achieved_goal, desired_goal):
        d = self._calc_distance(achieved_goal, desired_goal)
        return (d < self.threshold).astype(np.float32)


# class BaxterReacherEnv(gym.GoalEnv):
#     #TODO: Make this the base reacher env
#     """Simple implementation of a Baxter Reacher Env that is meant to work
#     with the real robot.
#     """
#
#     metadata = {'render.modes': ['human']}
#
#     def __init__(self,
#                  sim=False,
#                  state_type='ee_position',
#                  control = 'joint_positions',
#                  reward_type=REWARD.SPARSE,
#                  arm="right"):
#         self.sim = sim
#         self.arm = arm
#         self.state_type = state_type
#         self.reward_type = reward_type
#         self.goal = self._sample_goal()
#
#         if self.sim:
#             import pybullet as p
#             self._p = p
#             p.connect(p.Direct)
#         else:
#             # initialize ros node
#             rospy.init_node("reacher_env")
#         self.baxter = Baxter(sim=self.sim,
#                              arm=self.arm,
#                              control='joint_positions',
#                              state_type=state_type)
#         # goal threshold
#         self.threshold = 1e-2
#
#     def reset(self):
#         self.baxter.reset()
#         self.goal = self._sample_goal()
#         if self.sim:
#             self._env_setup()
#         return self._get_obs()
#
#     def seed(self):
#         # Implement
#         return
#
#     def step(self, action):
#         self._apply_action(action)
#         obs = self._get_obs()
#         ee_pos = obs[:3]
#         reward = self.compute_reward(ee_pos, self.goal)
#         return obs, reward, done, {}
#
#     def _get_obs(self):
#         ee_pose = self.baxter.get_ee_pose()
#         joint_angles = self.baxter.get_joint_angles()
#         achieved_goal = ee_pose[:3]
#         obs = np.array(ee_pose + joint_angles)
#         {
#             'observation': np.array(obs),
#             'achieved_goal': np.array(achieved_goal),
#             'desired_goal': self.goal,
#         }
#         return obs
#
#     def _apply_action(self, action):
#         self.baxter.apply_action(action)
#         self.baxter.control_rate.sleep()
#
#     def _is_success(self, achieved_goal, desired_goal):
#         pass
#
#     def compute_reward(self, achieved_goal, desired_goal, info):
#         dist = self._calc_distance(achieved_goal, desired_goal)
#         if self.reward_type  == REWARD.SPARSE:
#             return -(dist > self.threshold).astype(np.float32)
#         else:
#             return -dist
#
#     def _sample_goal(self):
#         # TODO: change bounds on goal sampling
#         x = np.random.uniform(.5, 1., 1)[0]
#         y = np.random.uniform(-1, 0, 1)[0]
#         z = np.random.uniform(-.3, .7, 1)[0]
#         if self.arm == 'left':
#             y *= -1
#         return [x, y, z]
#
#     def _calc_distance(self, goal_a, goal_b):
#         assert goal_a.shape == goal_b.shape
#         return np.linalg.norm(a,b, axis=-1)
#
#     def _is_success(self, achieved_goal, desired_goal):
#         d = self._calc_distance(achieved_goal, desired_goal)
#         return (d < self.threshold).astype(np.float32)
#
#     def render(self, mode='human', close=False):
#         if self.sim:
#             pass
#         else:
#             pass
#
#     # TODO Change action space and observaton space
#     @property
#     def action_space(self):
#         return Box(-np.inf, np.inf, (self.baxter.get_action_dimension(),))
#
#     @property
#     def observation_space(self):
#         return Box(-np.inf, np.inf, (self.baxter.get_action_dimension() + len(self.goal),))
