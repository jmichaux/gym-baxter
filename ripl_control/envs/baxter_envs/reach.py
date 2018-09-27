import random
import time

import numpy as np
import rospy

import gym
from gym import error, utils, spaces
from gym.spaces import Box
from gym.utils import seeding

from ripl_control.envs.baxter import *
from ripl_control.envs.robot_base_env import RobotBaseEnv

class REWARD(IntEnum):
    SPARSE = 0
    DENSE = 1

class BaxterReacherEnv(RobotBaseEnv):
    def __init__(self,
                 sim=False,
                 arm="right",
                 control='ee_pose',
                 goal_threshold=1e-2,
                 reward_type=REWARD.SPARSE,
                 n_actions=4,
                 discrete_actions=True):
        self.sim = sim
        if self.sim:
            import pybullet as p
            p.connect(p.Direct)
            self._dv = 0.005
            self._action_repeat = 1
        else:
            rospy.init_node("reacher_env")
            self._dv = 0.05
        self.discrete_actions = discrete_actions
        self._env_setup(sim, arm, control, goal_threshold, reward_type, initial_pose=None)
        super(BaxterReacherEnv, self).__init__(n_actions, n_substeps=20)

    def reset(self):
        self.baxter.reset()
        self.goal = self._sample_goal()
        return self._get_obs()

    def _env_setup(self, sim, arm, control, goal_threshold, reward_type, initial_pose=None):
        # create robot
        self.baxter = Baxter(sim=sim,
                             arm=arm,
                             control=control)
        # set reward type and goal threshold
        self.goal_threshold = goal_threshold
        self.reward_type = reward_type

    def _get_obs(self):
        ee_position = self.baxter.get_ee_position()
        joint_angles = self.baxter.get_joint_angles()
        achieved_goal = ee_position[:3]
        state = np.array(ee_position + joint_angles)
        obs = {
            'observation': np.array(state),
            'achieved_goal': np.array(achieved_goal),
            'desired_goal': self.goal,
        }
        return obs

    def _apply_action(self, action):
        if self.discrete_actions:
            pass
            dx = [0,-self._dv,self._dv,0,0,0,0][action]
            dy = [0,0,0,-self._dv,self._dv,0,0][action]
            dz = [0,0,0,0,0,-self._dv,self._dv][action]
        else:
            dx = action[0] * self._dv
            dy = action[1] * self._dv
            dz = action[2] * self._dv
        real_action = [dx, dy, dz, 0, 0, 0]
        if self.sim:
            for i in range(self._action_repeat):
                self.baxter_apply_action(real_action)
        else:
            self.baxter.apply_action(real_action)
            self.baxter.control_rate.sleep()

    def compute_reward(self, achieved_goal, desired_goal, info):
        dist = self._calc_distance(achieved_goal, desired_goal)
        if self.reward_type  == REWARD.SPARSE:
            return -(dist > self.goal_threshold).astype(np.float32)
        else:
            return -dist

    def _sample_goal(self):
        # TODO: change bounds on goal sampling
        x = np.random.uniform(.5, 1., 1)[0]
        y = np.random.uniform(-1, 0, 1)[0]
        z = np.random.uniform(-.3, .7, 1)[0]
        if self.baxter.arm_name == 'left':
            y *= -1
        return np.array([x, y, z])

    def _calc_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a-goal_b, axis=-1)

    def _is_success(self, achieved_goal, desired_goal):
        d = self._calc_distance(achieved_goal, desired_goal)
        return (d < self.goal_threshold).astype(np.float32)

    def _set_observation_space(self):
        """Returns the observation space
        """
        obs = self._get_obs()
        return spaces.Dict(dict(
            desired_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            achieved_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            observation=spaces.Box(-np.inf, np.inf, shape=obs['observation'].shape, dtype='float32')))

    def _set_action_space(self, n_actions):
        """Returns the action space
        """
        if self.discrete_actions:
            return spaces.Discrete(7)
        else:
            return spaces.Box(-1., 1., shape=(n_actions,), dtype='float32')
