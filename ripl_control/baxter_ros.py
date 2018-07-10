from __future__ import division
from enum import IntEnum
import numpy as np

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper

class Control(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2

class BaxterRos(object):
    def __init__(self, arms, control=None, rate=100.0, missed_cmds=20000.0):
        # create arm(s)
        self._arms = arms
        self.create_arms()

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
        # enable
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
        # elif arms == "right":
        #     self.right_arm = Limb("right")
        #     self.right_arm.gripper = Gripper("right")
        #     self.num_arms = 1
        # else:
        #     self.left_arm = Limb("left")
        #     self.left_arm.gripper = Gripper("left")
        #     self.num_arms = 1
        else:
            self.arm = Limb(self._arms)
            self.arm.gripper = Gripper(self._arms)
            self.num_arms = 1
        return

    def calibrate_grippers(self):
        if self._arms == "both":
            self.left_arm.gripper.calibrate()
            self.right_arm.gripper.calibrate()
        else:
            self.arm.gripper.calibrate()
        return

    def enable(self):
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    def shutdown(self):
        pass

    def get_joint_states(self):
        if self.control = Control.POSITION:
            return self.get_joint_angles()
        elif self.control = Control.VELOCITY:
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


class BaxterPyBullet(object):
    def __init__(self, control=None, limbs=None):
        pass

    def reset(self):
        pass

    def get_joint_angles(self):
        pass

if __name__ == "__main__":
    rospy.init_node("interface_test")
    baxter = BaxterRos("both")
