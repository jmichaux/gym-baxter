from __future__ import division
from enum import IntEnum
import numpy as np
import time

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper
from utils import transforms

class CONTROL(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2

class STATE(IntEnum):
    EE_POSITION = 0
    EE_POSE = 1
    JOINT_ANGLES = 2
    JOINT_VELOCITIES = 3
    JOINT_TORQUES = 4

class BaxterRos(object):
    def __init__(self, arms="both", control=None, state=None, rate=100.0, missed_cmds=20000.0):
        # create arm(s)
        self._arms = arms
        self.create_arms()

        # create joint dictionaries
        self.create_joint_dicts()

        # specify state
        # TODO: rewrite to handle multiple state types
        self.current_state = None
        self._robot_state = state

        # specify joint control mode
        self.set_control(control)

        # set joint command timeout and control rate
        self.rate = rate
        self.freq = 1 / rate
        self.missed_cmds = missed_cmds
        self.set_command_time_out()
        self.control_rate = rospy.Rate(self.rate)

        # reset starting state of robot
        self.reset()

    def set_control(self, control):
        """
        Sets the control type for Baxter
        """
        if control == "position" or control == None:
            self.control = CONTROL.POSITION
        elif control == "velocity":
            self.control = CONTROL.VELOCITY
        else:
            self.control = CONTROL.TORQUE
        return

    def set_command_time_out(self):
        if self._arms == "both":
            self.left_arm.set_command_timeout(self.freq * self.missed_cmds)
            self.right_arm.set_command_timeout(self.freq * self.missed_cmds)
        else:
            self.arm.set_command_timeout((1.0 / self.rate) * self.missed_cmds)
        return

    def reset(self):
        # enable robot
        self.enable()
        # move arms to ready positions
        self.set_ready_position()
        # if self._arms == "both":
        #     self.left_arm.move_to_neutral()
        #     self.right_arm.move_to_neutral()
        # else:
        #     self.arm.move_to_neutral()
        # calibrate grippers
        self.calibrate_grippers()
        # update state
        self.update_state()
        return

    def create_arms(self):
        """
        Create arm interface objects from Baxter. An arm consists
        of a Limb and its gripper.
        """
        if self._arms == "both":
            # create left arm
            self.left_arm = Limb("left")
            self.left_arm.gripper = Gripper("left")
            # create right arm
            self.right_arm = Limb("right")
            self.right_arm.gripper = Gripper("right")
            self.num_arms = 2
        elif self._arms == "left":
            # create left arm only
            self.arm = self.left_arm = Limb("left")
            self.arm.gripper = self.left_arm.gripper = Gripper("left")
            self._idle_arm = Limb("right")
            self.num_arms = 1
        else:
            # create right arm only
            self.arm = self.right_arm = Limb("right")
            self.arm.gripper = self.right_arm.gripper = Gripper("right")
            self._idle_arm = Limb("left")
            self.num_arms = 1
        return

    def calibrate_grippers(self):
        """
        (Blocking) Calibrates gripper(s) if not
        yet calibrated
        """
        if self._arms == "both":
            if not self.left_arm.gripper.calibrated():
                self.left_arm.gripper.calibrate()
            if not self.right_arm.gripper.calibrated():
                self.right_arm.gripper.calibrate()
        else:
            if not self.arm.gripper.calibrated():
                self.arm.gripper.calibrate()
        return

    def enable(self):
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    def shutdown(self):
        pass

    def get_state(self):
        """
        Returns the current state of the robot
        """
        if self._robot_state is None or self._robot_state == STATE.EE_POSITION:
            return self.get_ee_position()
        if self._robot_state == STATE.EE_POSE:
            return self.get_ee_pose()
        if self._robot_state == STATE.JOINT_ANGLES:
            return self.get_joint_angles()
        elif self._robot_state == STATE.JOINT_VELOCITIES:
            return self.get_joint_velocities()
        else:
            return self.get_joint_torques()
        pass

    def update_state(self):
        """
        Update the current state of the robot
        """
        self.current_state = self.get_state()
        return

    def get_ee_pose(self):
        """
        Returns a list of
        pose = [X, Y, Z, x, y, z, w]
        """
        if self._arms == "both":
            left_pose = self._left_ee_position() + self._left_ee_orientation()
            right_pose =  self._right_ee_position() + self._right_ee_orientation()
            return left_pose + right_pose
        else:
            return self.get_ee_position() + self.get_ee_orientation()

    def get_ee_position(self):
        """
        Returns a list of the position of the end effector(s)
        [X, Y, Z] or
        [X_left, Y_left, Z_left, X_right, Y_right, Z_right]
        """
        if self._arms == "both":
            return self._left_ee_position() + self._right_ee_position()
        else:
            return list(self.arm.endpoint_pose()['position'])

    def get_ee_orientation(self):
        """
        Returns a list of the orientations of the end effectors(s)
        [theta_x, theta_y, theta_z] or
        [the_x_l, theta_y_l, theta_z_l, the_x_r, theta_y_r, theta_z_r]
        """
        if self._arms == "both":
            return self._left_ee_orientation() + self._right_ee_orientation()
        else:
            # TODO: transform to Euler angles
            return list(self.arm.endpoint_pose()['orientation'])

    def _left_ee_position(self):
        return list(self.left_arm.endpoint_pose()['position'])

    def _right_ee_position(self):
        return list(self.right_arm.endpoint_pose()['position'])

    def _left_ee_orientation(self):
        return list(self.left_arm.endpoint_pose()['orientation'])

    def _right_ee_orientation(self):
        return list(self.right_arm.endpoint_pose()['orientation'])

    def get_joint_velocities(self):
        pass

    def get_joint_torques(self):
        pass

    def create_joint_dicts(self):
        self.joint_dict = self.create_joint_lookup_dict()
        self.joint_ranges = self.create_joint_range_dict()
        self.num_joints = len(self.joint_dict)
        return

    def create_joint_lookup_dict(self):
        """"
        Creates a dictionary that maps ints to joint names
        {int: joint name}
        """
        if self._arms == "both":
            joints = self.left_arm.joint_names() + self.right_arm.joint_names()
        else:
            joints = self.arm.joint_names()
        inds = range(len(joints))
        joint_dict = dict(zip(inds, joints))
        return joint_dict

    def create_joint_range_dict(self):
        joint_ranges = {
            'left_s0' : {'min': -1.7016, 'max': 1.7016},
            'left_s1' : {'min': -2.147, 'max': 2.147},
            'left_e0' : {'min': -3.0541, 'max': 3.0541 },
            'left_e1' : {'min': -0.05, 'max': 2.618},
            'left_w0' : {'min': -3.059, 'max': 3.059 },
            'left_w1' : {'min': -1.5707, 'max': 2.094},
            'left_w2' : {'min': -3.059, 'max': 3.059},
            'right_s0' : {'min': -1.7016, 'max': 1.7016},
            'right_s1' : {'min': -2.147, 'max': 2.147},
            'right_e0' : {'min': -3.0541, 'max': 3.0541},
            'right_e1' : {'min': -0.05, 'max': 2.618},
            'right_w0' : {'min': -3.059, 'max': 3.059 },
            'right_w1' : {'min': -1.5707, 'max': 2.094},
            'right_w2' : {'min': -3.059, 'max': 3.059 }}
        return joint_ranges

    def choose_random_action():
        pass

    def apply_action(self, action):
        """
        Apply a joint action
        Inputs:
            action - list or array of joint angles
        Blocking when moving the arm
        """
        assert(len(action) == len(self.joint_dict))
        if self._arms == "both":
            action_dict = self.create_action_dict(action)
            l_action_dict, r_action_dict = self.parse_action_dict(action_dict)
            # not blocking
            self.left_arm.set_joint_positions(l_action_dict)
            # blocking
            self.right_arm.move_to_joint_positions(r_action_dict)
        else:
            action_dict = self.create_action_dict(action)
            # blocking
            self.arm.move_to_joint_positions(action_dict)
        self.update_state()

    def parse_action_dict(self, action_dict):
        l_dict = {joint_name: action_dict[joint_name] for joint_name in self.left_arm.joint_names()}
        r_dict = {joint_name: action_dict[joint_name] for joint_name in self.right_arm.joint_names()}
        return l_dict, r_dict

    def create_action_dict(self, action):
        """
        Creates an action dictionary
        {joint_name: joint_angle}
        """
        action_dict = dict()
        for i, act in enumerate(action):
            joint_name = self.joint_dict[i]
            if act < self.joint_ranges[joint_name]['min']:
                act = self.joint_ranges[joint_name]['min']
            if act > self.joint_ranges[joint_name]['max']:
                act = self.joint_ranges[joint_name]['max']
            action_dict[joint_name] = act
        return action_dict

    def set_ready_position(self):
        angles = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
        if self._arms == "both":
            l_position = dict(zip(self.left_arm.joint_names(), angles))
            r_position = dict(zip(self.right_arm.joint_names(), angles))
            self.left_arm.set_joint_positions(l_position)
            self.right_arm.move_to_joint_positions(r_position)
        else:
            positions = dict(zip(self.arm.joint_names(), angles))
            self.arm.set_joint_positions(positions)
            self.tuck_idle_arm()

    def tuck_idle_arm(self):
        # tuck = {'left':  [-1.0, -2.07,  3.0, 2.55,  0.0, 0.01,  0.0],
        #         'right':  [1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0]}
        tuck = {'left':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0],
                'right':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0]}
        angles = tuck[self._idle_arm.name]
        positions = dict(zip(self._idle_arm.joint_names(), angles))
        self._idle_arm.move_to_joint_positions(positions)

    def untuck_idle_arm(self):
        # untuck = {'left':  [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50],
        #           'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]}
        angles = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
        positions = dict(zip(self._idle_arm.joint_names(), angles))
        self._idle_arm.set_joint_positions(positions)

    def move_to_ee_pose(self, pose):
        pass

    def move_to_ee_position(self, position):
        pass

    def move_joint_positions(self, joint_angles):
        pass

    def set_joint_velocities(self, joint_velocities):
        pass

    def set_joint_torques(self, joint_torques):
        pass

if __name__ == "__main__":
    rospy.init_node("interface_test")
    baxter = BaxterRos("left", state=STATE.EE_POSE)
