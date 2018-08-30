from __future__ import division
from enum import IntEnum
import os
import time

import numpy as np

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper
# import baxter_pybullet_interface
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
    EE = 3 #TODO MAKE SURE THIS WORKS WITH PYBULLET

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
                 control="positDion",
                 arm="right",
                 state_type=STATE.EE_POSE,
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

            # create arms
            self.create_arms(arm)

            # create joint dictionaries
            self.create_joint_dicts()

            # # state
            self.state_type = state_type
            self.state = self.get_state()

            # set joint command timeout and control rate
            self.rate = rate
            self.freq = 1 / rate
            self.missed_cmds = missed_cmds
            self.set_command_time_out()
            self.control_rate = rospy.Rate(self.rate)

            # reset robot
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
        elif control == "torque" or control == CONTROL.TORQUE:
            self.control = CONTROL.TORQUE
        else:
            self.control = CONTROL.EE
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
        # self.update_state()

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

    def create_arms(self, arm):
        """
        Create arm interface objects for Baxter.

        An arm consists of a Limb and its Gripper.

        Args
            arm (str): "right", "left", "both"
        """
        if arm not in ["right", "left", "both"]:
            raise ValueError("You must specify arm must as 'left', 'right', or 'both'")

        # number of arms
        if arm == "both":
            self.num_arms = 2
        else:
            self.num_arms = 1

        # number of degrees of freedom
        self.dof = self.calc_dof()

        # create arm objects
        if self.sim:
            self.left_arm = pybullet_interface.Limb(self.baxter_id, "left")
            # self.left_arm.gripper = pybullet_interface.Gripper("left")

            self.right_arm = pybullet_interface.Limb(self.baxter, "right")
            # self.right_arm.gripper = pybullet_interface.Gripper("right")
        else:
            self.left_arm = Limb("left")
            self.left_arm.gripper = Gripper("left")

            self.right_arm = Limb("right")
            self.right_arm.gripper = Gripper("right")

            if arm == "right":
                self.arm = self.right_arm
                self.arm.gripper = self.right_arm.gripper
            elif arm == "left":
                self.arm = self.left_arm
                self.arm.gripper = self.left_arm.gripper
        return

    def calc_dof(self):
        """
        Number of degrees of freedom
        """
        #TODO
        # rewrite to accomodate active and inactive joints
        if self.control == CONTROL.EE:
            return 6 * self.num_arms
        else:
            return 7 * self.num_arms

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
        Returns the current state of the real or simulated robot
        """
        if self.state_type == STATE.EE_POSE:
            return self.get_ee_pose()
        if self.state_type == STATE.JOINT_ANGLES:
            return self.get_joint_angles()
        elif self.state_type == STATE.JOINT_VELOCITIES:
            return self.get_joint_velocities()
        else:
            return self.get_joint_torques()

    def update_state(self):
        """
        Update the current state of the robot
        """
        self.state = self.get_state()
        return

    def move_to_ee_pose(self, pose, arm="right", blocking=True):
        """
        Move end effector to specified pose

        Args
            pose (list): [X, Y, Z, r, p, w]
            arm (string): "left" or "right"
        """
        joints = self.calc_ik(pose, arm)
        if arm is None:
            if self.num_arms == 2:
                raise ValueError("Must specify arg arm as 'left' or 'right' when planning with both arms.")
            if blocking:
                self.arm.move_to_joint_positions(joints)
            else:
                self.arm.set_joint_positions(joint)
        if arm == "left":
            if blocking:
                self.left_arm.move_to_joint_positions(joints)
            else:
                self.left_arm.set_joint_positions(joints)
        else:
            if blocking:
                self.right_arm.move_to_joint_positions(joints)
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
        if arm is None:
            if self.num_arms == 2:
                raise ValueError("Must specify arg arm as 'left' or 'right' when planning with both arms.")
        else:
            pos = self.get_ee_position(arm)
            orn = self.get_ee_orientation(arm, mode)
        return pos + orn

    def get_ee_position(self, arm=None):
        """
        Returns end effector position for specified arm.

        Returns the 3D cartesion coordinates of the end effector.
        Args
            arm (string): "left" or "right"
        Returns
            [X,Y,Z]
        """
        if arm is None:
            if self.num_arms == 2:
                raise ValueError("Must specify arg arm as 'left' or 'right' when planning with both arms.")
            else:
                return list(self.arm.endpoint_pose()['position'])
        elif arm == "left":
            return list(self.left_arm.endpoint_pose()['position'])
        elif arm == "right":
            return list(self.right_arm.endpoint_pose()['position'])
        else:
            raise ValueError("Arg arm should be 'left' or 'right'.")

    def get_ee_orientation(self, arm="right", mode=None):
        """
        Returns a list of the orientations of the end effectors(s)
        Args
            arm (string): "right" or "left"
            mode (string): specifies angle representation
        Returns
            orn (list): list of Euler angles or Quaternion
        """
        if arm is None:
            if self.num_arms == 2:
                raise ValueError("Must specify arg arm as 'left' or 'right' when planning with both arms,")
            else:
                orn = list(self.arm.endpoint_pose()['orientation'])
        elif arm == "left":
            orn = list(self.left_arm.endpoint_pose()['orientation'])
        elif arm == "right":
            orn = list(self.right_arm.endpoint_pose()['orientation'])
        else:
            raise ValueError("Arg arm should be 'left' or 'right'.")
        return list(p.getEulerFromQuaternion(orn))

    def get_joint_velocities(self, arm):
        pass

    def get_joint_torques(self, arm):
        pass

    def action_dimension(self):
        """
        Returns size of action
        """
        return self.dof

    def apply_action(self, action):
        """
        Apply a joint action

        Blocking on real robot

        Args
            action - list or tuple specifying end effector pose(6DOF * num_arms)
                    or joint angles (7DOF * num_arms)
        """
        verified, err = self._verify_action(action)
        if not verified:
            raise err

        if self.control == CONTROL.POSITION:
            self._apply_position_control(action)
        elif self.control == CONTROL.EE:
            self._apply_ee_control(action)
        elif self.control == CONTROL.VELOCITY:
            self._apply_velocity_control(action)
        else:
            self._apply_torque_control(action)

    def _verify_action(self, action):
        """
        Verify type and dimension of action

        Args
            action (list): list of floats len will vary depending on action type

        Returns
            bool: True if action is right dimension, false otherwise
        """
        # assert action is a list or tuple
        if not isinstance(action, (list, tuple)):
            return False, TypeError("Action must be a list or tuple.")
        if self.control == CONTROL.EE:
            if len(action) != 6*self.num_arms:
                return False, ValueError("Action must have len {}".format(6*self.num_arms))
        else:
            if len(action) != 7*self.num_arms:
                return False, ValueError("Action must have len {}".format(7*self.num_arms))
        return True, ""

    def clip_action(self):
        pass

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
            if self.num_arms == 1:
                action_dict = self.create_action_dict(action)
                # blocking
                self.arm.move_to_joint_positions(action_dict)
            else:
                action_dict = self.create_action_dict(action)
                l_action_dict, r_action_dict = self.parse_action_dict(action_dict)
                # not blocking
                self.left_arm.set_joint_positions(l_action_dict)
                # blocking
                self.right_arm.move_to_joint_positions(r_action_dict)
            self.update_state()
        return

    def _apply_ee_control(self, action):
        """
        Apply action to move Baxter end effector(s)

        Args
            action (list or tuple)
        """
        if self.sim:
            pass
        else:
            if self.num_arms == 1:
                self.move_to_ee_pose(action, arm=self.arm.name, blocking=True)
            else:
                left_action = action[:self.dof]
                right_action = action[self.dof:]
                self.move_to_ee_pose(left_action, arm=self.left_arm.name, blocking=False)
                self.move_to_ee_pose(right_action, arm=self.right_arm.name, blocking=True)
        return

    def calc_ik(self, ee_pose, arm):
        """
        Calculate inverse kinematics for a given end effector pose

        Args
            ee_pose (list or tuple of floats): 6 dimensional array specifying
                the position and orientation of the end effector
            arm (string): "right" or "left"

        Return
            joints (list): A list of joint angles
        """
        if arm is None:
            if self.num_arms == 2:
                raise ValueError("Must specify arg arm when both arms are active")
            else:
                arm = self.arm.get_name()
        if self.sim:
            joints = self._sim_ik(ee_pose, arm)
        else:
            joints = self._real_ik(ee_pose, arm)
        return joints

    def _sim_ik(self, ee_pose, arm):
        """
        (Sim) Calculate inverse kinematics for a given end effector pose


        Args:
            ee_pose (tuple or list): [pos, orn] of desired end effector pose
                pos - x,y,z
                orn - r,p,w
            arm (string): "right" or "left"
        Returns:
            joint_angles (list): List of joint angles
        """
        #TODO:
        #Assert that the correct joint angles change
        pass

    def _real_ik(self, ee_pose, arm):
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
            # rospy.logerr("Service call failed: %s" % (e,))
            return

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
        Set ready position for both arms.
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
            self.right_arm.move_to_neutral()
            self.left_arm.move_to_neutral()
        return

    def euler_to_quat(self, orn):
        # TODO: replace this function
        # TODO: assert orn is right shape
        return p.getQuaternionFromEuler(orn)

    def quat_to_euler(self, orn):
        # TODO: assert orn is right shape
        return p.getEulerFromQuaternion(orn)

if __name__ == "__main__":
    rospy.init_node("interface_test")
    baxter = Baxter("left", state=STATE.EE_POSE)
