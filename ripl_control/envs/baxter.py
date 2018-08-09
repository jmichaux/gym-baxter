from __future__ import division
from enum import IntEnum
import os
import time

import numpy as np

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
# from utils import transforms

import pybullet as p

class CONTROL(IntEnum):
    # 0
    VELOCITY = p.VELOCITY_CONTROL
    # 1
    TORQUE = p.TORQUE_CONTROL
    # 2
    POSITION = p.POSITION_CONTROL
    # 3
    EE = p.POSITION_CONTROL

class STATE(IntEnum):
    EE_POSITION = 0
    EE_POSE = 1
    JOINT_ANGLES = 2
    JOINT_VELOCITIES = 3
    JOINT_TORQUES = 4
    PIXELS = 5

class Baxter(object):
    def __init__(self,
                 sim=False,
                 arms="right",
                 time_step=1.0,
                 control="position",
                 state=None,
                 rate=100.0,
                 missed_cmds=20000.0):

        self.set_control(control)
        self.sim = sim

        if self.sim:
            print("hello")
            self.baxter_path =  os.path.expanduser("~") + baxter_path
            self.time_step = time_step
            # values taken from http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications#Peak_Torque
            self.max_velocity = 0.0
            self.max_force = 35.0
            self.max_gripper_force = 35.0
            self.left_finger_force = 0
            self.right_finger_force = 0
            self.finger_tip_force = 0.
        else:
            # set time step
            self.time_step = time_step

            # create arm(s)
            self._arms = arms
            self.create_arms()

            # create joint dictionaries
            self.create_joint_dicts()

            # specify state
            self.current_state = None
            self._robot_state = state

            # set joint command timeout and control rate
            self.rate = rate
            self.freq = 1 / rate
            self.missed_cmds = missed_cmds
            self.set_command_time_out()
            self.control_rate = rospy.Rate(self.rate)

            # reset starting state of robot
            # self.reset()

    def set_control(self, control):
        """
        Sets the control type for Baxter in simulation or the real robot
        Args: control - string or int

        """
        if control == "position" or control == CONTROL.POSITION or control == None:
            self.control = CONTROL.POSITION
        elif control == "velocity" or control == CONTROL.VELOCITY:
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
        if self.sim:
            self._reset_sim()
        else:
            self._reset_real()

    def _reset_real(self):
        # enable robot
        self.enable()
        # move arms to ready positions
        self.set_ready_position()
        # calibrate gripper
        self.calibrate_grippers()
        # update state
        self.update_state()

    def _reset_sim(self):
        '''Load Baxter, table, and tray URDF'''
        objects = [p.loadURDF(baxter_path, [0.00000,-0.000000, 1.00000,], useFixedBase=True)]
        objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
        self.baxter_id = objects[0]
        self.num_joints = p.getNumJoints(self.baxter_id)

        # Initial pose for left and right arms
        self.initial_pose = [
          # right arm
          [12, 0.005309502001432469],
          [13, -0.5409516930403337],
          [14, 0.004660885344502314],
          [15, 0.7525584394255346],
          [16, 0.0020039031898094538],
          [18, 1.2527114375915975],
          [19, -0.0049519009839707265],
          # right gripper
          [27, -0.005206632462850682],
          [29, -0.005206632462850682],
          # left arm
          [34, 0.006443994385763649],
          [35, -0.5429260025686881],
          [36, 0.0032076910770917574],
          [37, 0.7532434642513719],
          [38, -0.00378481197484175],
          [40, 1.2540471030663223],
          [41, -0.005206632462850682],
          # left gripper
          [49, -0.005206632462850682],
          [51, -0.005206632462850682]]

        # set position of arms
        for joint_index, joint_val in self.initial_pose:
            p.resetJointState(self.baxter_id, joint_index, joint_val)
            p.setJointMotorControl2(baxter, joint_index, p.POSITION_CONTROL, joint_val, force=self.max_force)

        # Get pose of end effectors
        self.left_ee_pos, self.left_ee_angle = self.get_ee_pose(end_effector="left")
        self.right_ee_pos, self.right_ee_angle = self.get_ee_pose(end_effector="right")

        self.motor_names = []
        self.motor_indices = []

        for i in range(self.num_joints):
          joint_info = p.getJointInfo(self.baxter_id, i)
          q_ind = joint_info[3]
          if q_ind > -1:
            self.motor_names.append(str(joint_info[1]))
            self.motor_indices.append(i)

        # lower limits, upper limits, joint ranges,
        # and rest poses for null space
        self.ll, self.ul, self.jr, self.rp = self.get_joint_ranges()

        # joint damping coefficients
        self.jd = None


    def create_arms(self):
        """
        Create arm interface objects from Baxter. An arm consists
        of a Limb and its gripper.
        """
        if self.sim:
            pass
        else:
            if self._arms == "both":
                # create left arm
                print("Creating both arms...")
                self.left_arm = Limb("left")
                self.left_arm.gripper = Gripper("left")
                # create right arm
                self.right_arm = Limb("right")
                self.right_arm.gripper = Gripper("right")
                self.num_arms = 2
            elif self._arms == "left":
                # create left arm only
                print("Creating left arm...")
                self.arm = self.left_arm = Limb("left")
                self.arm.gripper = self.left_arm.gripper = Gripper("left")
                self._idle_arm = Limb("right")
                self.num_arms = 1
            elif self._arms == "right":
                # create right arm only
                print("Creating right arm...")
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

    def update_state(self):
        """
        Update the current state of the robot
        """
        if self.sim:
            pass
        else:
            self.current_state = self.get_state()
        return

    def get_ee_pose(self):
        """
        Returns a list of
        pose = [X, Y, Z, x, y, z, w]
        """
        if self.sim:
            pass
        else:
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
        if self.sim:
            pass
        else:
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
        if self.sim:
            pass
        else:
            if self._arms == "both":
                return self._left_ee_orientation() + self._right_ee_orientation()
            else:
                # TODO: transform to Euler angles
                return list(self.arm.endpoint_pose()['orientation'])

    def _left_ee_position(self):
        if self.sim:
            pass
        else:
            return list(self.left_arm.endpoint_pose()['position'])

    def _right_ee_position(self):
        return list(self.right_arm.endpoint_pose()['position'])

    def _left_ee_orientation(self):
        if self.sim:
            pass
        else:
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
        if self.sim:
            pass
        else:
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

    def apply_action(self, action):
        """
        Apply a joint action
        Inputs:
            action - list or array of joint angles
        Blocking when moving the arm
        """
        if self.control == CONTROL.POSITION:
            self._apply_position_control(action)
        elif self.control == CONTROL.EE:
            self._apply_ee_control(action)
        elif self.control == CONTROL.VELOCITY:
            self._apply_velocity_control(action)
        else:
            self._apply_torque_control(action)

    def _apply_position_control(self, action):
        """
        Apply a joint action
        Inputs:
            action - list or array of joint angles
        Blocking when moving the arm
        """
        # TODO: check that action is of right type
        if self.sim:
            pass
        else:
            assert(len(action) == len(self.joint_dict))
            if self._arms == "both":
                print(True)
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
        return

    def _apply_ee_control(self, action):
        pass

    def _apply_velocity_control(self, action):
        # TODO: check that action is of right type
        if self.sim:
            pass
        else:
            pass
        return

    def _apply_torque_control(self, action):
        # TODO: check that action is of right type
        if self.sim:
            pass
        else:
            pass
        return

    def clip_action(self):
        pass

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
        idle_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print("Setting ready position")
        if self.sim:
            pass
        else:
            if self._arms == "both":
                # left arm
                l_position = dict(zip(self.left_arm.joint_names(), angles))
                self.left_arm.set_joint_positions(l_position)
                # right arm
                r_position = dict(zip(self.right_arm.joint_names(), angles))
                self.right_arm.move_to_joint_positions(r_position)
            else:
                #TODO: figure out why using set_joint_positions doesn't always send a command
                # active arm
                arm_position = dict(zip(self.arm.joint_names(), angles))
                self.arm.move_to_joint_positions(arm_position)
                # self.arm.set_joint_positions(arm_position)
                # idle arm
                idle_position = dict(zip(self._idle_arm.joint_names(), idle_angles))
                self._idle_arm.move_to_joint_positions(idle_position)
        return

    def tuck_idle_arm(self):
        if self.sim:
            pass
        else:
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
    baxter = Baxter("left", state=STATE.EE_POSE)
