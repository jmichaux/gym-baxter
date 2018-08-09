import os
import numpy as np
import pybullet as p
import time

import pybullet_data
# from baxter import *
#cid = p.connect(p.UDP,"192.168.86.100")
cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()


WORLD_OFFSET = 0.05

# Load plane
objects = [p.loadURDF("plane.urdf", 0,0,-0.93)]

# Load Baxter
urdf_path = "/home/ripl/ripl-control/ripl_control/envs/assets/baxter_robot/baxter_description/urdf/baxter.urdf"
baxter_pos = (0.0000, 0.0000000000, 0.00000000)
baxter_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
baxter_orientation = p.getQuaternionFromEuler([0, 0, 0])
objects = [p.loadURDF(urdf_path, basePosition=baxter_pos, baseOrientation=baxter_orientation, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE)]
baxter_id = objects[0]

# baxter params
max_velocity = 0.0
max_force = 35.0
max_gripper_force = 35.0
left_finger_force = 0
right_finger_force = 0
finger_tip_force = 0.

# set initial position and joint control
initial_pose = [
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

for joint_index, joint_val in initial_pose:
    p.resetJointState(baxter_id, joint_index, joint_val)
    p.setJointMotorControl2(baxter_id, joint_index, p.POSITION_CONTROL, joint_val)

target_pose = [
  # right arm
  [12, 0.00],
  [13, 0.00],
  [14, 0.00],
  [15, 0.00],
  [16, 0.00],
  [18, 0.00],
  [19, 0.00],
  # right gripper
  [27, -0.005206632462850682],
  [29, -0.005206632462850682],
  # left arm
  [34, 0.00],
  [35, 0.00],
  [36, 0.00],
  [37, 0.00],
  [38, 0.00],
  [40, 0.00],
  [41, 0.00],
  # left gripper
  [49, -0.005206632462850682],
  [51, -0.005206632462850682]]


def setMotorsAngleInRealTimestep(self, motorTargetAngles, motorTargetTime=0, delayTime):
    if(motorTargetTime == 0):
        _joint_targetPos = np.array(motorTargetAngles)
        for i in range(_joint_number):
            p.setJointMotorControl2(bodyIndex=_robot, jointIndex=_joint_id[i], controlMode=p.POSITION_CONTROL,
                                    targetPosition=_joint_targetPos[i],
                                    positionGain=_kp, velocityGain=_kd, force=_torque, maxVelocity=_max_velocity)
        time.sleep(delayTime)
    else:
        _joint_currentPos = _joint_targetPos
        _joint_targetPos = np.array(motorTargetAngles)
        for i in range(_joint_number):
            dydt = (_joint_targetPos-_joint_currentPos)/motorTargetTime
        internalTime = 0.0
        reft = time.time()
        while internalTime < motorTargetTime:
            internalTime = time.time() - reft
            for i in range(_joint_number):
                p.setJointMotorControl2(bodyIndex=_robot, jointIndex=_joint_id[i], controlMode=p.POSITION_CONTROL,
                                        targetPosition=_joint_currentPos[i] + dydt[i] * internalTime,
                                        positionGain=_kp, velocityGain=_kd, force=_torque, maxVelocity=_max_velocity)


p.setGravity(0.000000,0.000000,-10.000000)

while True:
# while (time.time() < ref_time+running_time):
  # p.setGravity(0,0,-10)
  p.stepSimulation()



# def setMotorsAngleInFixedTimestep(self, motorTargetAngles, motorTargetTime, delayTime):
#     if(motorTargetTime == 0):
#         _joint_targetPos = np.array(motorTargetAngles)
#         for i in range(_joint_number):
#             p.setJointMotorControl2(bodyIndex=_robot, jointIndex=_joint_id[i], controlMode=p.POSITION_CONTROL,
#                                     targetPosition=_joint_targetPos[i],
#                                     positionGain=_kp, velocityGain=_kd, force=_torque, maxVelocity=_max_velocity)
#             p.stepSimulation()
#             time.sleep(_timeStep)
#     else:
#         _joint_currentPos = _joint_targetPos
#         _joint_targetPos = np.array(motorTargetAngles)
#         for i in range(_joint_number):
#             dydt = (_joint_targetPos-_joint_currentPos)/motorTargetTime
#         internalTime = 0.0
#         while internalTime < motorTargetTime:
#             internalTime += _timeStep
#             for i in range(_joint_number):
#                 p.setJointMotorControl2(bodyIndex=_robot, jointIndex=_joint_id[i], controlMode=p.POSITION_CONTROL,
#                                         targetPosition=_joint_currentPos[i] + dydt[i] * internalTime,
#                                         positionGain=_kp, velocityGain=_kd, force=_torque, maxVelocity=_max_velocity)
#             p.stepSimulation()
# time.sleep(_timeStep)


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
