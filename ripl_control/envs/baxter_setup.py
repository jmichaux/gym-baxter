import os
import numpy as np
import pybullet as p
import time

import pybullet_data
from baxter import *
#cid = p.connect(p.UDP,"192.168.86.100")
cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()


WORLD_OFFSET = 0.05

# Load skybox
# TODO: Adjust lighting
# plane_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/plane.urdf"
# objects = [p.loadURDF("plane.urdf")]

# pos_0 = (0, 0, WORLD_OFFSET)
# orientation_0 = p.getQuaternionFromEuler([0, 0, 0])
# objects = [p.loadURDF(plane_path, pos_0, orientation_0)]
#
# pos_1 = (5, 0, 0)
# orientation_1 = p.getQuaternionFromEuler([0, -np.pi/2, 0])
# objects = [p.loadURDF(plane_path, pos_1, orientation_1)]

# pos_2 = (-5, 0, 0)
# orientation_2 = p.getQuaternionFromEuler([0, np.pi/2, 0])
# objects = [p.loadURDF(plane_path, pos_2, orientation_2)]

# pos_3 = (0, 5, 0)
# orientation_3 = p.getQuaternionFromEuler([np.pi/2, 0, 0])
# objects = [p.loadURDF(plane_path, pos_3, orientation_3)]

# pos_4 = (0, -5, 0)
# orientation_4 = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
# objects = [p.loadURDF(plane_path, pos_4, orientation_4)]


# urdf_path = "/home/ripl/self_balancing_robot/urdf/simple.urdf"
# objects = [p.loadURDF(urdf_path, (0,0,.02))]

# Load Baxter
urdf_path = "/home/ripl/ripl-control/ripl_control/assets/baxter_robot/baxter_description/urdf/baxter.urdf"
baxter_pos = (0.0000, 0.0000000000, 0.00000000)
baxter_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
baxter_orientation = p.getQuaternionFromEuler([0, 0, 0])
objects = [p.loadURDF(urdf_path, basePosition=baxter_pos, baseOrientation=baxter_orientation, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE)]
# objects = [p.loadURDF(urdf_path, basePosition=baxter_pos, baseOrientation=baxter_orientation)]
baxter = objects[0]
#
# # joints
# baxter_joints = []
# head = [5, 7]
#
# for i in range(p.getNumJoints(baxter)):
#     baxter_joints.append(p.getJointInfo(baxter, i)[1].decode("utf-8"))
#
# # jointPositions=[ -1.000000, -1.000000, 1.000000, 1.570793, 1.000000, -1.036725, 0.000001 ]
# baxter_joints = [
#   # right arm
#   [12, 0.005309502001432469],
#   [13, -0.5409516930403337],
#   [16, 0.0020039031898094538],
#   [14, 0.004660885344502314],
#   [15, 0.7525584394255346],
#   [18, np.pi/2],
#   # [18, 1.2527114375915975],
#   [19, -0.0049519009839707265],
#   # right gripper
#   [27, 0.02],
#   [29, -0.02],
#   # left arm
#   [34, 0.006443994385763649],
#   [35, -0.5429260025686881],
#   [36, 0.0032076910770917574],
#   [37, 0.7532434642513719],
#   [38, -0.00378481197484175],
#   [40, np.pi/2],
#   # [40, 1.2540471030663223],
#   [41, -0.005206632462850682],
#   # left gripper
#   [49, 0.02],
#   [51, -0.02]]




# set position of arms
# for joint_index, joint_val in baxter_joints:
#     p.resetJointState(baxter, joint_index, joint_val)
#     # p.setJointMotorControl2(baxter, jointIndex, p.POSITION_CONTROL, jointVal, 0)
#     p.setJointMotorControl2(baxter, joint_index, p.POSITION_CONTROL, joint_val, 0)


# p.resetJointState(baxter, 40, np.pi/2)
# p.setJointMotorControl2(baxter, 40, p.POSITION_CONTROL, np.pi/2, 0)
# p.resetJointState(baxter, 18, np.pi/2)
# p.setJointMotorControl2(baxter, 18, p.POSITION_CONTROL, np.pi/2, 0)



# p.addUserDebugText("",[0,0,0.1],textColorRGB=[1,0,0],textSize=1.5,parentObjectUniqueId=baxter, parentLinkIndex=25)
# p.addUserDebugLine([0,0,0],[1.5,0,0],[1,0,0],parentObjectUniqueId=baxter, parentLinkIndex=25)
# p.addUserDebugLine([0,0,0],[0,1.5,0],[0,1,0],parentObjectUniqueId=baxter, parentLinkIndex=25)
# p.addUserDebugLine([0,0,0],[0,0,1.5],[0,0,1],parentObjectUniqueId=baxter, parentLinkIndex=25)
#
# # add debug axes to right end effector
# p.addUserDebugText("",[0,0,0.1],textColorRGB=[1,0,0],textSize=1.5,parentObjectUniqueId=baxter, parentLinkIndex=41)
# p.addUserDebugLine([0,0,0],[1.5,0,0],[1,0,0],parentObjectUniqueId=baxter, parentLinkIndex=41)
# p.addUserDebugLine([0,0,0],[0,1.5,0],[0,1,0],parentObjectUniqueId=baxter, parentLinkIndex=41)
# p.addUserDebugLine([0,0,0],[0,0,1.5],[0,0,1],parentObjectUniqueId=baxter, parentLinkIndex=41)

# add debug axes to right end effector
# p.addUserDebugText("",[0,0,0.1],textColorRGB=[1,0,0],textSize=1.5,parentObjectUniqueId=baxter, parentLinkIndex=42)
# p.addUserDebugLine([0,0,0],[1.5,0,0],[0,1,0],parentObjectUniqueId=baxter, parentLinkIndex=42)
# p.addUserDebugLine([0,0,0],[0,1.5,0],[0,1,0],parentObjectUniqueId=baxter, parentLinkIndex=42)
# p.addUserDebugLine([0,0,0],[0,0,1.5],[0,1,0],parentObjectUniqueId=baxter, parentLinkIndex=42)
# Load workspace

# Large workspace
# workspace_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/workspace.urdf"
# objects = [p.loadURDF(workspace_path, basePosition=(1.90000,-0.500000, 0.400000), baseOrientation=(0.000000,0.000000,0.707107,0.707107), useFixedBase=True)]

# small workspace
# workspace_small_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/workspace_small.urdf"
# objects = [p.loadURDF(workspace_small_path, basePosition=(2.1500, 0.000000, 0.400000), baseOrientation=(0.000000,0.000000,0.707107,0.707107), useFixedBase=True)]
#
# # Load target
# target_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/target.urdf"
# objects = [p.loadURDF(target_path, basePosition=(2.00, 0.00000, 1.400000), useFixedBase=True)]

# load blocks


# Load puck

p.setGravity(0.000000,0.000000,-10.000000)
# p.setGravity(0,0,-10)

# p.setRealTimeSimulation(1)
# ref_time = time.time()
#
# # This demonstrates that I am actually planning in the world coordinates
# pos = (2.00, 0.00000, 1.400000)
# orn = p.getQuaternionFromEuler([0, np.pi,0])
# joint_poses = p.calculateInverseKinematics(baxter, 48, pos, None)
#
# num_joints = p.getNumJoints(baxter)
# for i in range(num_joints):
#   joint_info = p.getJointInfo(baxter, i)
#   q = joint_info[3]
#   print(q, q-7)
#   if q > -1:
#     p.resetJointState(baxter, i, joint_poses[q - 7])
#     p.setJointMotorControl2(baxter, i, p.POSITION_CONTROL, joint_poses[q - 7], 0)
#
# running_time = 360 # seconds
while True:
# while (time.time() < ref_time+running_time):
  # p.setGravity(0,0,-10)
  p.stepSimulation()
