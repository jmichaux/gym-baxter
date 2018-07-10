import os, inspect


import pybullet as p
import numpy as np
import copy 
import math
import pybullet_data
# import baxter_data


class Baxter():
  def __init__(self, baxter_path="/code/envs/gym-baxter/data/baxter_robot/baxter_description/urdf/baxter.urdf", 
               use_ik=True,
               use_simulation=True,
               use_nullspace=True,
               planar_grasp=True,
               time_step=0.01):

    self.baxter_path =  os.path.expanduser("~") + baxter_path
    self.time_step = time_step
    # values taken from http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications#Peak_Torque
    self.max_velocity = 0.0
    self.max_force = 35.0 
    self.max_gripper_force = 35.0
    self.left_finger_force = 0
    self.right_finger_force = 0
    self.finger_tip_force = 0.

    # Inverse kinematics
    self.use_ik = use_ik
    self.use_simulation = use_simulation
    self.use_nullspace = use_nullspace
    self.planar_grasp = planar_grasp
    # Set wrist orientation for gripper
    if self.planar_grasp:
      self.default_orn = p.getQuaternionFromEuler([0, np.pi,0])
    else:
      self.default_orn = None

    # End effectors
    self.left_ee_index = 48
    self.right_ee_index = 26

    self.reset()

  def reset(self):
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
    
  def get_joint_ranges(self, include_fixed=False):
    lower_limits = []
    upper_limits = []
    joint_ranges = []
    rest_poses = []

    num_joints = p.getNumeJoints(self.baxter_id)

    for i in range(num_joints):
      joint_info = p.getJointInfo(self.baxter_id, i)
      
      if joint_info[3] > -1:
        ll, ul = joint_info[8:10]
        jr = ul - ll

        rp = p.getJointState(self.baxter_id, i)[0]

        lower_limits.append(-2)
        upper_limits.append(2)
        joint_ranges.append(2)
        rest_poses.append(rp)
    return lower_limits, upper_limits, joint_ranges, rest_poses

  def get_action_dimension(self):
    if self.use_ik:
      return len(self.motor_indices)
    return 12 # x,y,z, roll/pitch/yaw euler angles of two end effectors

  def get_observation_dimension(self):
    return len(self.get_observation())

  def get_observation(self):
    '''Get pose of the left and right end effectors'''
    l_ee_pos, l_ee_orn = self.get_ee_pose(end_effector="left")
    r_ee_pos, r_ee_orn = self.get_ee_pose(end_effector="right")
    observation = [l_ee_pos + l_ee_orn, r_ee_pos + r_ee_orn]
    return observation

  def get_ee_pose(self, end_effector="left"):
    """Get pose of end effector"""
    if end_effector == "left":
      state = p.getLinkState(self.baxter_id, self.left_ee_index)
    elif end_effector == "right":
      state = p.getLinkState(self.baxter_id, self.right_ee_index)
    else: # raise error
      print("Entered incorrect string. Must be 'right' or 'left'")
      exit(1)
    pos = list(state[0])
    orn = state[1]
    euler = list(p.getEulerFromQuaternion(orn))
    return pos, euler

  def apply_action(self, motor_commands):
    """
    motor commands is an array of len ten.
    """
    #TODO: Define workspace for Baxter
    # right arm
    delta_right_arm = motor_commands[0:4]
    right_gripper = motor_commands[4]
    self.right_ee_pos[0] = self.right_ee_pos[0] + delta_right_arm[0]
    self.right_ee_pos[1] = self.right_ee_pos[1] + delta_right_arm[1]
    self.right_ee_pos[2] = self.right_ee_pos[2] + delta_right_arm[2]

    # left arm
    delta_left_arm = motor_commands[5:9]
    left_gripper = motor_commands[9]
    self.left_ee_pos[0] = self.left_ee_pos[0] + delta_left_arm[0]
    self.left_ee_pos[1] = self.left_ee_pos[1] + delta_left_arm[1]
    self.left_ee_pos[2] = self.left_ee_pos[2] + delta_left_arm[2]

    # calculate ik
    if self.use_ik:
      joint_poses = self.calc_ik(self.left_ee_index, self.left_ee_pos)
      self.set_motors()
      joint_poses = self.calc_ik(self.right_ee_index, self.right_ee_pos)
      self.set_motors()

    # right end effector angle and fingers
    p.setJointMotorControl2(self.baxter_id, 7, p.POSITION_CONTROL, targetPosition=self.endEffectorAngle, force=self.max_force)
    p.setJointMotorControl2(self.baxter_id, 27, p.POSITION_CONTROL, targetPosition=right_gripper, force=self.fingerAForce)
    p.setJointMotorControl2(self.baxter_id, 29, p.POSITION_CONTROL, targetPosition=right_gripper, force=self.fingerBForce)
    # left end effector angle and fingers
    p.setJointMotorControl2(self.baxter_id, 7, p.POSITION_CONTROL, targetPosition=self.endEffectorAngle, force=self.max_force)
    p.setJointMotorControl2(self.baxter_id, 49, p.POSITION_CONTROL, targetPosition=left_gripper, force=self.fingerAForce)
    p.setJointMotorControl2(self.baxter_id, 51, p.POSITION_CONTROL, targetPosition=left_gripper, force=self.fingerBForce)
    return

  def update_ee(self, motor_commands):
    new_pose = []
    dx = motor_commands[0]
    dy = motor_commands[1]
    dz = motor_commands[2]
    da = motor_commands[3]
    gripper = motor_commands[4]
    return

  def calc_ik(self, ee_index, pos):
    if self.use_nullspace:
      joint_poses = p.calculateInverseKinematics(self.baxter_id, ee_index,
                                                 pos, self.default_orn, 
                                                 self.ll, self.ul, self.jr, self.rp)
    else:
      joint_poses = p.calculateInverseKinematics(self.baxter_id, ee_index,
                                                 pos, self.default_orn, jointDamping=self.jd)
    return joint_poses   

  def set_motors(self, joint_poses):
    return

  def close_gripper(self):
    return

  def get_observation_dict(self):
    return

    
