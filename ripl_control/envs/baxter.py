from __future__ import division
from enum import IntEnum
import os
import time

import numpy as np

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper
# from pybullet_interface import Limb, Gripper
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
# from utils import transforms

import pybullet as p

class CONTROL(IntEnum):
    # 0
    VELOCITY = 0
    # 1
    TORQUE = 1
    # 2 (Requires IK)
    POSITION = 2
    EE = 2

class STATE(IntEnum):
    JOINT_VELOCITIES = 0
    JOINT_TORQUES = 1
    JOINT_ANGLES = 2
    EE_POSE = 3
    PIXELS = 4

class Baxter(object):
    def __init__(self,
                 sim=False,
                 time_step=1.0,
                 control="position",
                 state=None,
                 rate=100.0,
                 missed_cmds=20000.0):

        self.set_control(control)
        self.sim = sim

        if self.sim:
            self.baxter_path =  os.path.expanduser("~") + baxter_path
            self.time_step = time_step
            # values taken from
            # http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications#Peak_Torque
            # TODO: verify these numbers
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
            self.reset()

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
        self.left_arm.set_command_timeout(self.freq * self.missed_cmds)
        self.right_arm.set_command_timeout(self.freq * self.missed_cmds)
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
        # Load assets for Baxter
        objects = [p.loadURDF(baxter_path, [0.00000,0.000000, 1.00000,], useFixedBase=True)]
        # objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
        self.baxter_id = objects[0]
        self.num_joints = p.getNumJoints(self.baxter_id)

        # move arms to ready position
        self.set_ready_position()

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
            self.left_arm = pybullet_interface.Limb("left")
            # self.left_arm.gripper = pybullet_interface.Gripper("left")

            self.right_arm = pybullet_interface.Limb("right")
            # self.right_arm.gripper = pybullet_interface.Gripper("right")
        else:
            self.left_arm = Limb("left")
            self.left_arm.gripper = Gripper("left")

            self.right_arm = Limb("right")
            self.right_arm.gripper = Gripper("right")
        return

    def calibrate_grippers(self):
        """
        (Blocking) Calibrates gripper(s) if not
        yet calibrated
        """
        if not self.left_arm.gripper.calibrated():
            self.left_arm.gripper.calibrate()
        if not self.right_arm.gripper.calibrated():
            self.right_arm.gripper.calibrate()
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

    def move_to_ee_pose(self, pose, arm="right"):
        """
        (Blocking) Move end effector to specified pose

        Args
            pose (list): [X, Y, Z, r, p, w]
            arm (string): "left" or "right"
        """
        joints = self.calc_ik(pose)
        if arm == "left":
            self.left_arm.move_to_joint_positions(joints)
        else:
            self.right_arm.move_to_joint_positions(joints)
        return

    def set_ee_pose(self, pose, arm="right"):
        """
        (Not Blocking) Move end effector to specified pose

        Args
            pose (list): [X, Y, Z, r, p, w]
            arm (string): "left" or "right"
        """
        joints = self.get_ik(pose)
        if arm == "left":
            self.left_arm.set_joint_positions(joints)
        else:
            self.right_arm.set_joint_positions(joints)
        return

    def get_ee_pose(self, arm="right", mode=None):
        """
        Returns end effector pose for specified arm.

        End effector pose is a list of values corresponding to the 3D cartesion coordinates
        and roll (r), pitch (p), and yaw (w) Euler angles.

        Args
            arm (string): "right" or "left"
            mode (string): "Quaternion" or "quaterion" or "quat"
        Returns
            pose (list): Euler angles [X,Y,Z,r,p,w] or Quaternion [X,Y,Z,x,y,z,w]
        """
        if arm == "left":
            pos = self.get_ee_position("left")
            orn = self.get_ee_orientation("left", mode=mode)
        else:
            pos = self.get_ee_position("right")
            orn = self.get_ee_orientation("right", mode=mode)
        return pos + orn

    def get_ee_position(self, arm="right"):
        """
        Returns end effector position for specified arm.

        Returns the 3D cartesion coordinates of the end effector.
        Args
            arm (string): "left" or "right"
        Returns
            [X,Y,Z]
        """
        if arm == "left":
            return list(self.left_arm.endpoint_pose()['position'])
        elif arm == "right":
            return list(self.right_arm.endpoint_pose()['position'])

    def get_ee_orientation(self, arm="right", mode=None):
        """
        Returns a list of the orientations of the end effectors(s)
        Args
            arm (string): "right" or "left"
            mode (string): specifies angle representation
        Returns
            orn (list): list of Euler angles or Quaternion
        """
        if arm == "left":
            orn = self._left_ee_orientation()
        else:
            orn = self._right_ee_orientation()
        if mode in ["Quaternion", "quaternion", "quat"]:
                return orn
        else:
            return list(p.getEulerFromQuaternion(orn))

    def _left_ee_position(self):
        """
        Returns the position of the left end effector
        """
        if self.sim:
            pass
        else:
            return list(self.left_arm.endpoint_pose()['position'])

    def _right_ee_position(self):
        """
        Returns the position of the right end effector
        """
        if self.sim:
            pass
        return list(self.right_arm.endpoint_pose()['position'])

    def _left_ee_orientation(self):
        """
        Returns the orientation of the left end effector
        """
        if self.sim:
            pass
        else:
            return list(self.left_arm.endpoint_pose()['orientation'])

    def _right_ee_orientation(self):
        """
        Returns the orientation of the right end effector
        """
        if self.sim:
            pass
        return list(self.right_arm.endpoint_pose()['orientation'])

    def get_joint_velocities(self, arm):
        pass

    def get_joint_torques(self, arm):
        pass

    def create_joint_dicts(self):
        self.joint_dict = self.create_joint_lookup_dict()
        self.joint_ranges = self.create_joint_range_dict()
        return

    def create_joint_lookup_dict(self):
        """"
        Creates a dictionary that maps ints to joint names
        {int: joint name}
        """
        if self.sim:
            pass
        else:
            joints = self.left_arm.joint_names() + self.right_arm.joint_names()
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
        """
        Sets ready position for both arms
        """
        if self.sim:
            # Initial pose for left and right arms
            initial_pose = [
              # right arm
              [12, 0.00],
              [13, -0.55],
              [14, 0.00],
              [15, 0.75],
              [16, 0.00],
              [18, 1.26],
              [19, 0.00],
              # right gripper
              [27, -0.0052],
              [29, -0.0052],
              # left arm
              [34, 0.00],
              [35, -0.55],
              [36, 0.00],
              [37, 0.75],
              [38, 0.00],
              [40, 1.26],
              [41, 0.00],
              # left gripper
              [49, -0.0052],
              [51, -0.0052]]

            # set position of arms
            for joint_index, joint_val in initial_pose:
                p.resetJointState(self.baxter_id, joint_index, joint_val)
                p.setJointMotorControl2(baxter, joint_index, p.POSITION_CONTROL, joint_val, force=self.max_force)
        else:
            angles = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
            # left arm
            l_position = dict(zip(self.left_arm.joint_names(), angles))
            self.left_arm.set_joint_positions(l_position)
            # right arm
            r_position = dict(zip(self.right_arm.joint_names(), angles))
            self.right_arm.move_to_joint_positions(r_position)
        return

    def calc_ik(self, ee_pose, arm="right"):
        """
        Calculate inverse kinematics for a given end effector pose
        """
        if self.sim:
            joints = self._sim_ik(ee_pose, arm)
        else:
            joints = self._real_ik(ee_pose, arm)
        return joints

    def _sim_ik(self, ee_pose, arm="right"):
        """
        (Sim) Calculate inverse kinematics for a given end effector pose


        Args:
            ee_pose (tuple or list): [pos, orn] of desired end effector pose
                pos - x,y,z
                orn - r,p,w
        Returns:
            joint_angles (list): List of joint angles
        """
        #TODO:
        #Assert that the correct joint angles change
        pass

    def _real_ik(self, ee_pose, arm="right"):
        """
        (Real) Calculate inverse kinematics for a given end effector pose

        Args:
            ee_pose (tuple or list): [pos, orn] of desired end effector pose
                pos - x,y,z
                orn - r,p,w
        Returns:
            joint_angles (list): List of joint angles
        """
        pos = ee_pose[:3]
        orn = ee_pose[3:]
        orn = self.euler_to_quat(orn)

        ns = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        ik_pose = PoseStamped()
        ik_pose.pose.position.x = pos[0]
        ik_pose.pose.position.y = pos[1]
        ik_pose.pose.position.z = pos[2]
        ik_pose.pose.orientation.x = orn[0]
        ik_pose.pose.orientation.y = orn[1]
        ik_pose.pose.orientation.z = orn[2]
        ik_pose.pose.orientation.w = orn[3]
        ik_pose.header = hdr
        ikreq.pose_stamp.append(ik_pose)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return {}
        #
        # # Check if result valid, and type of seed ultimately used to get solution
        # # convert rospy's string representation of uint8[]'s to int's
        # resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
        #                            resp.result_type)
        # print resp_seeds
        # if (resp_seeds[0] != resp.RESULT_INVALID):
        #     seed_str = {
        #                 ikreq.SEED_USER: 'User Provided Seed',
        #                 ikreq.SEED_CURRENT: 'Current Joint Angles',
        #                 ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
        #                }.get(resp_seeds[0], 'None')
        #     print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
        #           (seed_str,))
        #     # Format solution into Limb API-compatible dictionary
        #     limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #     print "\nIK Joint Solution:\n", limb_joints
        #     print "------------------"
        #     print "Response Message:\n", resp
        # else:
        #     print("INVALID POSE - No Valid Joint Solution Found.")
        # # limb_interface.move_to_joint_positions(limb_joints)
        # return limb_joints

    def euler_to_quat(self, orn):
        # TODO: replace this function
        # TODO: assert orn is right shape
        return p.getQuaternionFromEuler(orn)

    def quat_to_euler(self, orn):
        # TODO: assert orn is right shape
        return p.getEulerFromQuaternion(orn)

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



if __name__ == "__main__":
    rospy.init_node("interface_test")
    baxter = Baxter("left", state=STATE.EE_POSE)
