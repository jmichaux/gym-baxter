from __future__ import division
from enum import IntEnum
import os
import time
import numpy as np

import rospy
from baxter_pykdl import baxter_kinematics
import baxter_interface
from baxter_interface import CHECK_VERSION, Limb, Gripper
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

# TODO: implement transforms and remove this import
import pybullet as p
import baxter_pybullet_interface as pybullet_interface

class CONTROL(IntEnum):
    VELOCITY = 0
    TORQUE = 1
    POSITION = 2
    EE = 3

class Baxter(object):
    """
    Baxter interface for doing RL experiments on real and simulated
    robots.
    """
    def __init__(self,
                 sim=False,
                 config=None,
                 time_step=1.0,
                 rate=100.0,
                 missed_cmds=20000.0):
        """
        Initialize Baxter Interface

        Args:
            sim (bool): True specifies using the PyBullet simulator
                False specifies using the real robot
            arm (str): 'right' or 'left'
            control (int): Specifies control type
                TODO: change to str ['ee', 'position', 'velocity']
            config (class): Specifies joint ranges, ik null space paramaters, etc
            time_step (float):
                TODO: probably get rid of this
            rate (float):
            missed_cmds (float):
        """
        self.sim = sim
        self.dof = {
            'left': {'ee': 6, 'position': 7, 'velocity': 7},
            'right': {'ee': 6, 'position': 7, 'velocity': 7},
            'both': {'ee': 12, 'position': 14, 'velocity': 14}}
            }

        if self.sim:
            self.baxter_path = "/home/ripl/ripl-control/ripl_control/envs/assets/baxter_robot/baxter_description/urdf/baxter.urdf"
            # self.baxter_path = "/assets/baxter_robot/baxter_description/urdf/baxter.urdf"
            self.time_step = time_step
        else:
            self.rate = rate
            self.freq = 1 / rate
            self.missed_cmds = missed_cmds
        self.config=config
        self.initial_setup()

    def set_control(self, control):
        """
        Sets the control type for Baxter in simulation or the real robot
        Args: control - string or int
        """
        if control in ["position", "joint_positions", "joint_angles", CONTROL.POSITION]  or control == None:
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

    def initial_setup(self):
        if self.sim:
            # load baxter
            objects = [p.loadURDF(self.baxter_path, useFixedBase=True)]
            self.baxter_id = objects[0]
            print("baxter_id: ", self.baxter_id)
            if self.config:
                self.use_nullspace = True
                self.use_orientation = True
                self.ll = self.config.nullspace.ll
                self.ul = self.config.nullspace.ul
                self.jr = self.config.nullspace.jr
                self.rp = self.config.nullspace.rp
            else:
                self.use_nullspace = False
            self.create_arms()
        else:
            self.create_arms()
            self.set_command_time_out()
            self.control_rate = rospy.Rate(self.rate)

    def reset(self, initial_pose=None):
        if self.sim:
            self._reset_sim(initial_pose)
        else:
            self._reset_real(initial_pose)
        # create joint dictionaries
        # TODO: WHY DO I DO THIS AGAIN?
        self.create_joint_dicts(arm="right")
        self.create_joint_dicts(arm="left")

    def _reset_real(self, initial_pose=None):
        self.enable()
        if initial_pose is None:
            self.set_ready_position()
        else:
            pass
        self.calibrate_grippers()

    def _reset_sim(self, control_type=None, initial_pose=None):
        # set control type
        if control_type == 'position' or control_type == 'ee' or control_type is None:
            control_mode = p.POSITION_CONTROL
        elif control_type == 'velocity':
            control_mode = p.VELOCITY_CONTROL
        elif control_mode == 'torque':
            control_mode = p.TORQUE_CONTROL
        else:
            raise ValueError("control_type must be in ['position', 'ee', 'velocity', 'torque']." )
        # set initial position
        if initial_pose:
            for joint_index, joint_val in initial_pose:
                p.resetJointState(0, joint_index, joint_val)
                p.setJointMotorControl2(self.baxter_id,
                                        jointIndex=joint_index,
                                        controlMode=control_mode,
                                        targetPosition=joint_val)
        else:
            num_joints = p.getNumJoints(self.baxter_id)
            for joint_index in range(num_joints):
                p.setJointMotorControl2(self.baxter_id,
                                        joint_index,
                                        control_mode,
                                        maxForce=self.config.max_force)
        return

    def create_arms(self):
        """
        Create arm interface objects for Baxter.

        An arm consists of a Limb and its Gripper.
        """
        # create arm objects
        if self.sim:
            self.left_arm = pybullet_interface.Limb(self.baxter_id, "left", self.config)
            # self.left_arm.gripper = pybullet_interface.Gripper("left")

            self.right_arm = pybullet_interface.Limb(self.baxter_id, "right", self.config)
            # self.right_arm.gripper = pybullet_interface.Gripper("right")
        else:
            self.left_arm = Limb("left")
            self.left_arm.gripper = Gripper("left")
            self.left_arm.kin = baxter_kinematics("left")

            self.right_arm = Limb("right")
            self.right_arm.gripper = Gripper("right")
            self.right_arm.kin = baxter_kinematics("right")
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

    def move_to_ee_pose(self, arm, pose, blocking=True):
        """
        Move end effector to specified pose

        Args
            arm (string): "left" or "right" or "both"
            pose (list):
                if arm == 'left' or arm == 'right':
                    pose = [X, Y, Z, r, p, w]
                else:
                    pose = left_pose + right _pose
        """
        if arm == "both":
            left_pose = pose[:6]
            right_pose = pose[6:]
            left_joints = self.ik('left', left_joints)
            right_joints = self.ik('right', right_joints)
            if blocking:
                self.left_arm.move_to_joint_positions(left_joints)
                self.right_arm.move_to_joint_positions(right_joints)
            else:
                self.left_arm.set_joint_positions(left_joints)
                self.right_arm.set_joint_positions(right_joints)
        elif arm == "left":
            joints = self.ik(arm, pose)
            if blocking:
                self.left_arm.move_to_joint_positions(joints)
            else:
                self.left_arm.set_joint_positions(joints)
        elif arm == "right":
            if arm == "right":
                if blocking:
                    self.right_arm.move_to_joint_positions(joints)
                else:
                    self.right_arm.set_joint_positions(joints)
        else:
            raise ValueError("Arm must be 'right', 'left', or 'both'")
        return

    def get_ee_pose(self, arm, mode=None):
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
        pos = self.get_ee_position(arm)
        orn = self.get_ee_orientation(arm, mode)
        return pos + orn

    def get_ee_position(self, arm):
        """
        Returns end effector position for specified arm.

        Returns the 3D cartesion coordinates of the end effector.
        Args
            arm (string): "left" or "right" or "both"
        Returns
            [X,Y,Z]
        """
        if arm not in ['left', 'right', 'both']:
            raise ValueError("Arg arm should be 'left' or 'right' or 'both'.")
        if arm == "left":
            return list(self.left_arm.endpoint_pose()['position'])
        elif arm == "right":
            return list(self.right_arm.endpoint_pose()['position'])
        elif arm == "both":
            return list(self.left_arm.endpoint_pose()['position']) + list(self.right_arm.endpoint_pose()['position'])

    def get_ee_orientation(self, arm, mode=None):
        """
        Returns a list of the orientations of the end effectors(s)
        Args
            arm (string): "right" or "left" or "both"
            mode (string): specifies angle representation
        Returns
            orn (list): list of Euler angles or Quaternion
        """
        if arm not in ['left', 'right', 'both']:
            raise ValueError("Arg arm should be 'left' or 'right'.")
        if arm == "left":
            orn = list(self.left_arm.endpoint_pose()['orientation'])
        elif arm == "right":
            orn = list(self.right_arm.endpoint_pose()['orientation'])
        elif arm == "both":
            orn = list(self.left_arm.endpoint_pose()['orientation']) + list(self.right_arm.endpoint_pose()['orientation'])
        return list(p.getEulerFromQuaternion(orn))

    def get_joint_angles(self, arm):
        """
        Get joint angles for specified arm

        Args
            arm(strin): "right" or "left" or "both"

        Returns
            joint_angles (list): List of joint angles starting from the right_s0 ('right_upper_shoulder')
                and going down the kinematic tree to the end effector.
        """
        if arm == "left":
            joint_angles = self.left_arm.joint_angles()
        elif arm == "right":
            joint_angles = self.right_arm.joint_angles()
        elif arm == "both":
            joint_angles = self.left_arm.joint_angles() + self.right_arm.joint_angles()
        else:
            raise ValueError("Arg arm should be 'left' or 'right' or 'both'.")
        if not self.sim:
            joint_angles = joint_angles.values()
        return joint_angles

    def get_joint_velocities(self, arm):
        """
        Get joint velocites for specified arm
        """
        if arm == "left":
            return self.left_arm.joint_velocities()
        elif arm == "right":
            return self.right_arm.joint_velocities()
        elif arm == "both":
            return self.left_arm.joint_velocities() + self.right_arm.joint_velocities()
        else:
            raise ValueError("Arg arm should be 'left' or 'right' or 'both'.")
        if not self.sim:
            joint_velocities = joint_velocities.values()
        return joint_velocities

    def get_joint_efforts(self, arm):
        """
        Get joint torques for specified arm
        """
        if arm == "left":
            return self.left_arm.joint_effort()
        elif arm == "right":
            return self.right_arm.joint_efforts()
        elif arm == "both":
            return self.left_arm.joint_efforts() + self.right_arm.joint_efforts()
        else:
            raise ValueError("Arg arm should be 'left' or 'right' or 'both'.")
        if not self.sim:
            joint_efforts = joint_efforts.values()
        return

    def apply_action(self, arm, control_type, action):
        """
        Apply a joint action

        Blocking on real robot

        Args
            action - list or tuple specifying end effector pose(6DOF * num_arms)
                    or joint angles (7DOF * num_arms)
        """
        # verify action
        verified, err = self._verify_action(arm, control_type, action)
        if not verified:
            raise err
        # execute action
        if control_type == 'position':
            self._apply_position_control(arm, action)
        elif control_type == 'ee':
            self._apply_ee_control(arm, action)
        elif control_type == 'velocity':
            self._apply_velocity_control(arm, action)
        else:
            self._apply_torque_control(arm, action)

    def _verify_action(self, arm, control_type, action):
        """
        Verify type and dimension of action

        Args
            arm (str): "left" or "right" or "both"
            action (list): list of floats len will vary depending on action type

        Returns
            bool: True if action is right dimension, false otherwise
        """
        if arm not in ["left", "right", "both"]:
            return False, ValueError("Arg arm must be string")
        if control_type not in ['ee', 'position', 'velocity', 'torque']:
            return False, ValueError("Arg control_type must be one of ['ee', 'position', 'velocity', 'torque']")
        if not isinstance(action, (list, tuple, np.ndarray)):
            return False, TypeError("Action must be a list or tuple.")
        if len(action) != self.dof[arm][control_type]:
            return False, ValueError("Action must have len {}".format(self.dof * num_arms))
        return True, ""

    def _clip_action(self, action):
        pass

    def get_arm(self, arm):
        if arm == 'right':
            return self.right_arm
        elif arm == 'left':
            return self.left_arm

    def _apply_torque_control(self, arm, action):
        """
        As of right now, setting joint torques does not
        does not command the robot as expected
        """
        raise NotImplementedError('Cannot apply torque control. Try using joint position or velocity control')

    def _apply_velocity_control(self, arm, action):
        """
        Apply velocity control to a given arm

        Args
            arm (str): 'right' or 'left'
            action (list, tuple, or numpy array) of len 7
        """
        if self.sim:
            pass
        else:
            if arm == 'left':
                action_dict = self
                self.right_arm.set_joint_velocities
            if arm == 'right':
                pass
            if arm == 'both':
                pass
        return

    def _apply_position_control(self, arm, action):
        """
        Apply a joint action
        Inputs:
            action - list or array of joint angles
        Blocking when moving the arm
        """
        action = list(action)
        if self.sim:
            if arm == 'left':
                self.left_arm.move_to_joint_positions(action)
            elif arm == 'right':
                self.right_arm.move_to_joint_positions(action)
            elif arm == 'both':
                joint_indices = self.left_arm.indices + self.right_arm.indices
                self.left_arm.move_to_joint_positions(action, joint_indices)
        else:
            if arm == 'both':
                    l_action = action[:7]
                    r_action = action[7:]
                    l_action_dict = self.create_action_dict('left', l_action)
                    r_action_dict = self.create_action_dict('right', r_action)
                    self.left_arm.set_joint_positions(l_action_dict)
                    self.right_arm.move_to_joint_positions(r_action_dict)
            if arm == 'left':
                if self.sim:
                    self.left_arm.move_to_joint_positions(action)
                else:
                    action_dict = self.create_action_dict(arm, action)
                    self.left_arm.set_joint_positions(action_dict)
            if arm == 'right':
                if self.sim:
                    self.right_arm.move_to_joint_positions(action)
                else:
                    action_dict = self.create_action_dict(arm, action)
                    self.right_arm.set_joint_positions(action_dict)
        self.update_state()
        return

    def _apply_ee_control(self, arm, action):
        """
        Apply action to move Baxter end effector(s)

        Args
            action (list, tuple, or numpy array)
        """
        action = list(action)
        if arm == 'left' or arm == 'right':
            self.move_to_ee_pose(arm, action)
        elif arm == 'both':
            if self.sim:
                joint_indices = self.left_arm.indices + self.right_arm.indices
                self.left_arm.move_to_joint_positions(actions, joint_indices)
            else:
                pass
        return

    def fk(self, arm, joints):
        """
        Calculate forward kinematics

        Args
            arm (str): 'right' or 'left'
            joints (list): list of joint angles

        Returns
            pose(list): [x,y,z,r,p,w]
        """
        if arm == 'right':
            pose = list(self.right_arm.kin.forward_position_kinematics(joints))
        elif arm == 'left':
            pose = list(self.left_arm.kin.forward_position_kinematics(joints))
        else:
            raise ValueError("Arg arm must be 'right' or 'left'")
        return pose[:3] + list(self.quat_to_euler(pose[3:]))

    def _ik(self, arm, pos, orn=None, seed=None):
        """
        Calculate inverse kinematics

        Args
            arm (str): 'right' or 'left'
            pos (list): [X, Y, Z]
            orn (list): [r, p, w]
            seed (int): for setting random seed

        Returns
            joint angles (list): A list of joint angles

        Note: This will probably fail if orn is not included
        """
        if orn is not None:
            orn = list(self.euler_to_quat(orn))
        if arm == 'right':
            joint_angles = self.right_arm.kin.inverse_kinematics(pos, orn)
        elif arm == 'left':
            joint_angles = self.left_arm.kin.inverse_kinematics(pos, orn, seed)
        else:
            raise ValueError("Arg arm must be 'right' or 'left'")
        return list(joint_angles.tolist())

    def ik(self, arm, ee_pose):
        """
        Calculate inverse kinematics for a given end effector pose

        Args
            ee_pose (list or tuple of floats): 6 dimensional array specifying
                the position and orientation of the end effector
            arm (string): "right" or "left"

        Return
            joints (list): A list of joint angles
        """
        if self.sim:
            joints = self._sim_ik(arm, ee_pose)
        else:
            joints = self._real_ik(arm, ee_pose)
        if not joints:
            print("IK failed. Try running again or changing the pose.")
        return joints

    def _sim_ik(self, arm, ee_pose):
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
        if arm == 'right':
            ee_index = self.right_arm.ee_index
        elif arm == 'left':
            ee_index = self.left_arm.ee_index
        elif arm == 'both':
            pass
        else:
            raise ValueError("Arg arm must be 'left', 'right', or 'both'.")
        pos = ee_pose[:3]
        orn = ee_pose[3:]
        if (self.use_nullspace==1):
            if (self.use_orientation==1):
                joints = p.calculateInverseKinematics(self.baxter_id, ee_index, pos, orn, self.ll, self.ul, self.jr, self.rp)
            else:
                joints = p.calculateInverseKinematics(self.baxter_id, ee_index, pos, lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr, restPoses=self.rp)
        else:
            if (self.use_orientation==1):
                joints = p.calculateInverseKinematics(self.baxter_id, ee_index, pos, orn,jointDamping=self.jd)
            else:
                joints = p.calculateInverseKinematics(self.baxter_id, ee_index, pos)
        return joints

    def _real_ik(self, arm, ee_pose):
        """
        (Real) Calculate inverse kinematics for a given end effector pose

        Args:
            ee_pose (tuple or list): [pos, orn] of desired end effector pose
                pos - x,y,z
                orn - r,p,w
        Returns:
            joint_angles (dict): Dictionary containing {'joint_name': joint_angle}
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

    def create_joint_dicts(self, arm):
        """
        Creates dictionaries for joint names and ranges
        """
        self.joint_dict = self.create_joint_lookup_dict(arm)
        self.joint_ranges = self.create_joint_range_dict(arm)
        return

    def create_joint_lookup_dict(self, arm):
        """"
        Creates a dictionary that maps ints to joint names
        {int: joint name}
        """
        if arm == "right":
            joints = self.right_arm.joint_names()
        elif arm == "left":
            joints = self.left_arm.joint_names()
        elif arm == "both":
            joints = self.left_arm.joint_names() + self.right_arm.joint_names()
        inds = range(len(joints))
        joint_dict = dict(zip(inds, joints))
        return joint_dict

    def create_joint_range_dict(self, arm):
        if arm == "right":
            joint_ranges = {
                'right_s0' : {'min': -1.7016, 'max': 1.7016},
                'right_s1' : {'min': -2.147, 'max': 2.147},
                'right_e0' : {'min': -3.0541, 'max': 3.0541},
                'right_e1' : {'min': -0.05, 'max': 2.618},
                'right_w0' : {'min': -3.059, 'max': 3.059 },
                'right_w1' : {'min': -1.5707, 'max': 2.094},
                'right_w2' : {'min': -3.059, 'max': 3.059 }}
        elif arm == "left":
            joint_ranges = {
                'left_s0' : {'min': -1.7016, 'max': 1.7016},
                'left_s1' : {'min': -2.147, 'max': 2.147},
                'left_e0' : {'min': -3.0541, 'max': 3.0541 },
                'left_e1' : {'min': -0.05, 'max': 2.618},
                'left_w0' : {'min': -3.059, 'max': 3.059 },
                'left_w1' : {'min': -1.5707, 'max': 2.094},
                'left_w2' : {'min': -3.059, 'max': 3.059}}
        elif arm == "both":
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
        else:
            raise ValueError("Arg arm must be 'right', 'left', or 'both'.")
        return joint_ranges

    def parse_action_dict(self, action_dict):
        l_dict = {joint_name: action_dict[joint_name] for joint_name in self.left_arm.joint_names()}
        r_dict = {joint_name: action_dict[joint_name] for joint_name in self.right_arm.joint_names()}
        return l_dict, r_dict

    def create_action_dict(self, arm, action):
        """
        Creates an action dictionary
        {joint_name: joint_angle}
        """
        if self.control == CONTROL.POSITION:
            if arm == 'left':
                joint_ranges = self.joint_position_ranges['left']
            else:
                joint_ranges = self.joint_position_ranges['right']
        if self.control == CONTROL.VELOCITY:
            if arm == 'left':
                joint_ranges = self.joint_velocity_ranges['left']
            else:
                joint_ranges = self.joint_velocity_ranges['right']
        action_dict = dict()
        for i, act in enumerate(action):
            joint_name = self.joint_dict[i]
            if act < joint_ranges[joint_name]['min']:
                act = joint_ranges[joint_name]['min']
            if act > joint_ranges[joint_name]['max']:
                act = joint_ranges[joint_name]['max']
            action_dict[joint_name] = act
        return action_dict

    def set_ready_position(self):
        """
        Set ready position for both arms.
        """
        if self.sim:
            pass
        else:
            self.right_arm.move_to_neutral()
            self.left_arm.move_to_neutral()
        return

    def euler_to_quat(self, orn):
        # TODO: replace this function
        return p.getQuaternionFromEuler(orn)

    def quat_to_euler(self, orn):
        return p.getEulerFromQuaternion(orn)

if __name__ == "__main__":
    rospy.init_node("interface_test")
    baxter = Baxter()
    baxter.reset()
