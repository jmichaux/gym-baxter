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

# TODO: implement transforms and remove this import
import pybullet as p

class CONTROL(IntEnum):
    VELOCITY = 0
    TORQUE = 1
    POSITION = 2
    EE = 3

class Baxter(object):
    def __init__(self,
                 sim=False,
                 arm="right",
                 control=CONTROL.EE,
                 time_step=1.0,
                 rate=100.0,
                 missed_cmds=20000.0):

        self.set_control(control)
        self.sim = sim
        self.arm_name = arm
        if self.arm_name == "both":
            self.num_arms = 2
        else:
            self.num_arms = 1
        self.dof = self.calc_dof()

        if self.sim:
            import pybullet as p
            import baxter_pybullet_interface as pybullet_interface
            self.baxter_urdf =  "/assets/baxter_robot/baxter_description/urdf/baxter.urdf"
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
            self.rate = rate
            self.freq = 1 / rate
            self.missed_cmds = missed_cmds
        self.create_arms()

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

    def reset(self):
        if self.sim:
            self._reset_sim()
            self.create_arms()
        else:
            self.create_arms()
            # set joint command timeout and control rate
            self.set_command_time_out()
            self.control_rate = rospy.Rate(self.rate)
            self._reset_real()
        # create joint dictionaries
        self.create_joint_dicts(arm="right")
        # self.create_joint_dicts(arm="left")

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
        '''Load Baxter URDF'''
        # Load assets for Baxter
        objects = [p.loadURDF(self.baxter_urdf, [0.00000,0.000000, 1.00000,], useFixedBase=True)]
        self.baxter_id = objects[0]
        self.num_joints = p.getNumJoints(self.baxter_id)

        # move arms to ready position
        # self.set_ready_position()

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
        Create arm interface objects for Baxter.

        An arm consists of a Limb and its Gripper.

        Args
            arm (str): "right", "left", "both"
        """
        # create arm objects
        if self.sim:
            self.left_arm = pybullet_interface.Limb(self.baxter_id, "left")
            # self.left_arm.gripper = pybullet_interface.Gripper("left")

            self.right_arm = pybullet_interface.Limb(self.baxter_id, "right")
            # self.right_arm.gripper = pybullet_interface.Gripper("right")
        else:
            self.left_arm = Limb("left")
            self.left_arm.gripper = Gripper("left")

            self.right_arm = Limb("right")
            self.right_arm.gripper = Gripper("right")
        return

    def calc_dof(self):
        """
        Number of degrees of freedom
        """
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

    def move_to_ee_pose(self, arm, pose, blocking=True):
        """
        Move end effector to specified pose

        Args
            pose (list): [X, Y, Z, r, p, w]
            arm (string): "left" or "right"
        """
        joints = self.calc_ik(arm, pose)
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
        if arm == "left":
            return list(self.left_arm.endpoint_pose()['position'])
        elif arm == "right":
            return list(self.right_arm.endpoint_pose()['position'])
        elif arm == "both":
            return list(self.left_arm.endpoint_pose()['position']) + list(self.right_arm.endpoint_pose()['position'])
        else:
            raise ValueError("Arg arm should be 'left' or 'right' or 'both'.")

    def get_ee_orientation(self, arm, mode=None):
        """
        Returns a list of the orientations of the end effectors(s)
        Args
            arm (string): "right" or "left" or "both"
            mode (string): specifies angle representation
        Returns
            orn (list): list of Euler angles or Quaternion
        """
        if arm == "left":
            orn = list(self.left_arm.endpoint_pose()['orientation'])
        elif arm == "right":
            orn = list(self.right_arm.endpoint_pose()['orientation'])
        elif arm == "both":
            orn = list(self.left_arm.endpoint_pose()['orientation']) + list(self.right_arm.endpoint_pose()['orientation'])
        else:
            raise ValueError("Arg arm should be 'left' or 'right'.")
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

    # def get_action_dimension(self):
    #     """
    #     Returns size of action
    #     """
    #     return self.dof * self.num_arms

    def apply_action(self, arm, action):
        """
        Apply a joint action

        Blocking on real robot

        Args
            action - list or tuple specifying end effector pose(6DOF * num_arms)
                    or joint angles (7DOF * num_arms)
        """
        # verify action
        verified, err = self._verify_action(arm, action)
        if not verified:
            raise err
        # execute action
        if self.control == CONTROL.POSITION:
            self._apply_position_control(arm, action)
        elif self.control == CONTROL.EE:
            self._apply_ee_control(arm, action)
        elif self.control == CONTROL.VELOCITY:
            self._apply_velocity_control(arm, action)
        else:
            self._apply_torque_control(arm, action)

    def _verify_action(self, arm, action):
        """
        Verify type and dimension of action

        Args
            arm (str): "left" or "right" or "both"
            action (list): list of floats len will vary depending on action type

        Returns
            bool: True if action is right dimension, false otherwise
        """
        if arm not in ["left", "right", "both"]:
            return False, ValueError("Arg name must be string")
        if not isinstance(action, (list, tuple, np.ndarray)):
            return False, TypeError("Action must be a list or tuple.")
        # check action has right size
        if arm == "both":
            num_arms = 2
        else:
            num_arms = 1
        if len(action) != self.dof * num_arms:
            return False, ValueError("Action must have len {}".format(self.dof * num_arms))
        return True, ""

    def _clip_action(self, action):
        pass

    def get_arm(self, arm):
        if arm == 'right':
            return self.right_arm
        elif arm == 'left':
            return self.left_arm

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
        if self.sim:
            if arm == 'left':
                self.left_arm.move_to_joint_positions(action)
            elif arm == 'right':
                self.right_arm.move_to_joint_positions(action)
            elif arm == 'both':
                joint_indices = self.left_arm.indices + self.right_arm.indices
                self.left_arm.move_to_joint_positions(actions, joint_indices)
        else:
            if arm == 'left' or arm == 'right':
                self.move_to_ee_pose(arm, action)
            elif arm == 'both':
                pass
            else:
                pass
        return
        # if self.sim:
        #     pass
        # else:
        #     if self.num_arms == 1:
        #         self.move_to_ee_pose(action, arm=self.arm.name, blocking=True)
        #     else:
        #         left_action = action[:self.dof]
        #         right_action = action[self.dof:]
        #         self.move_to_ee_pose(left_action, arm=self.left_arm.name, blocking=False)
        #         self.move_to_ee_pose(right_action, arm=self.right_arm.name, blocking=True)
        # return

    def calc_ik(self, arm, ee_pose):
        """
        Calculate inverse kinematics for a given end effector pose

        Args
            ee_pose (list or tuple of floats): 6 dimensional array specifying
                the position and orientation of the end effector
            arm (string): "right" or "left"

        Return
            joints (list): A list of joint angles
        """
        print(arm)
        print(ee_pose)
        new_pose = np.array(self.get_ee_pose(arm)) + np.array(ee_pose)
        if self.sim:
            joints = self._sim_ik(arm, new_pose)
        else:
            joints = self._real_ik(arm, new_pose)
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
        else:
            ee_index = self.left_arm.ee_index
        pos = ee_pose[:3]
        orn = ee_pose[3:]
        if (self.useNullSpace==1):
            if (self.useOrientation==1):
                joints = p.calculateInverseKinematics(self.baxter_id, ee_index, pos, orn, self.ll, self.ul, self.jr, self.rp)
            else:
                joints = p.calculateInverseKinematics(self.baxter_id, ee_index, pos, lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr, restPoses=self.rp)
        else:
            if (self.useOrientation==1):
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

    def _apply_velocity_control(self, action):
        return

    def _apply_torque_control(self, action):
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
        if self.sim:
            pass
        else:
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
        if self.sim:
            pass
        else:
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
        if arm == 'left':
            pass

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
            pass
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
    baxter = Baxter()
    baxter.reset()
