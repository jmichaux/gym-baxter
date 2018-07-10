from __future__ import division
from enum import IntEnum
import numpy as np
import time

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper

class Control(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2

class State(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2

class BaxterRos(object):
    def __init__(self, arms, control=None, state=None, rate=100.0, missed_cmds=20000.0):
        # create arm(s)
        self._arms = arms
        self.create_arms()

        # create joint dictionary
        # self.create_joint_dict()

        # specify joint control mode
        self.set_control(control)

        # set joint command timeout
        self.rate = rate
        self.freq = 1 / rate
        self.missed_cmds = missed_cmds
        self.set_command_time_out()

        # rest starting state of robot
        self.reset()

    def set_control(self, control):
        if control == "position" or control == None:
            self.control = Control.POSITION
        elif control == "velocity":
            self.control = Control.VELOCITY
        else:
            self.control = Control.TORQUE
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
        # move arms to Neutral position
        if self._arms == "both":
            self.left_arm.move_to_neutral()
            self.right_arm.move_to_neutral()
        else:
            self.arm.move_to_neutral()
        # calibrate grippers
        self.calibrate_grippers()
        return

    def take_action(self, action):
        pass

    def single_arm_action(self, action):
        pass

    def create_arms(self):
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
            self.num_arms = 1
        else:
            # create right arm only
            self.arm = self.right_arm = Limb("right")
            self.arm.gripper = self.right_arm.gripper = Gripper("right")
            self.num_arms = 1
        return

    def calibrate_grippers(self):
        """
        (Blocking)
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

    def get_joint_states(self):
        if self.state == State.POSITION:
            return self.get_joint_angles()
        elif self.state == State.VELOCITY:
            return self.get_joint_velocities()
        else:
            return self.get_joint_torques()
        pass

    def get_joint_angles(self):
        pass

    def get_joint_velocities(self):
        pass

    def get_joint_torques(self):
        pass

    def get_ee_states(self):
        pass

    def create_joint_dict(self):
        if self._arms == "both":
            joints = self.left_arm.joint_names() + self.right_arm.joint_names()
        else:
            joints = self.arm.joint_names()
        inds = range(len(joints))
        self.joint_dict = dict(zip(inds, joints))
        return


if __name__ == "__main__":
    rospy.init_node("interface_test")
    baxter = BaxterRos("both")

    r_joints_1 = {'right_e0': 0,
              'right_e1': 0,
              'right_s0': 0,
              'right_s1': 0,
              'right_w0': 0,
              'right_w1': 0,
              'right_w2': 0}

    l_joints_1 = {'left_e0': 0,
              'left_e1': 0,
              'left_s0': 0,
              'left_s1': 0,
              'left_w0': 0,
              'left_w1': 0,
              'left_w2': 0}

    r_joints_2 = {'right_e0': -0.0011504855909140602,
                'right_e1': 0.7620049563820793,
                'right_s0': -0.0011504855909140602,
                'right_s1': -0.540344732532637,
                'right_w0': -0.0011504855909140602,
                'right_w1': 1.2448254093690132,
                'right_w2': -0.0019174759848567672}

    l_joints_2 = {'left_e0': -0.0019174759848567672,
                 'left_e1': 0.7531845668517382,
                 'left_s0': 0.0007669903939427069,
                 'left_s1': -0.54686415088115,
                 'left_w0': -0.0015339807878854137,
                 'left_w1': 1.2563302652781538,
                 'left_w2': 0.0015339807878854137}



    # baxter.right_arm.move_to_joint_positions(r_joints_1)
    # arms.move_to_joint_positions(r_joints_1)
    # baxter.left_arm.set_joint_positions(l_joints_1)
