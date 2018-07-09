import copy
import sys


import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
import baxter_interface


class Baxter(object):
    def __init__(self, calibrate_grippers=False):
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        # both arms
        self.both_arms = MoveGroupCommander("both_arms")
        # left arm
        self.left_arm = MoveGroupCommander("left_arm")
        self.left_gripper = baxter_interface.Gripper("left")
        # right arm
        self.right_arm = MoveGroupCommander("right_arm")
        self.right_gripper = baxter_interface.Gripper("right")

        if calibrate_grippers:
            self.calibrate_grippers()

    def reset(self, ):
        # move to ready position
        return

    def get_current_pose(self):
        return

    def get_joint_values(self):
        return

    def calibrate_grippers(self):
        self.left_gripper.calibrate()
        self.right_gripper.calibrate()
        return

    def pick(self):
        return

    def place(self):
        return


class Arm(object):
    def __init__(self, arm):
        self.gripper = baxter_interface.gripper.Gripper("left")

    def create_arm(self):
        pass

    def close_gripper(self):
        pass

    def open_gripper(self):
        pass

    def move_
