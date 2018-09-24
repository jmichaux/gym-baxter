import numpy as np
import pybullet as p

class Limb(object):
    """
    PyBullet interface class for a limb on the Baxter robot.
    """
    def __init__(self, robot_id, name):
        # set robot id
        self.robot_id = robot_id
        self.joints = self.init_joints()
        self.name = name

        if self.name == 'right':
            self._ee_index = 26
            self.ros_to_pb = {'right_s0' : 12,
                              'right_s1' : 13,
                              'right_e0' : 14,
                              'right_e1' : 15,
                              'right_w0' : 16,
                              'right_w1' : 18,
                              'right_w2' : 19}

            self.pb_to_ros = {12 : 'right_s0',
                              13 : 'right_s1',
                              14 : 'right_e0',
                              15 : 'right_e1',
                              16 : 'right_w0',
                              18 : 'right_w1',
                              19 : 'right_w2'}
        else:
            self._ee_index = 48
            self.ros_to_pb = {'left_s0' : 12,
                              'left_s1' : 13,
                              'left_e0' : 14,
                              'left_e1' : 15,
                              'left_w0' : 16,
                              'left_w1' : 18,
                              'left_w2' : 19}

            self.pb_to_ros = {34 : 'left_s0',
                              35 : 'left_s1',
                              36 : 'left_e0',
                              37 : 'left_e1',
                              38 : 'left_w0',
                              40 : 'left_w1',
                              41 : 'left_w2'}


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


    def joint_names(self):
        return self.pb_to_ros.values()

    def joint_angle(self, joint):
        if joint not in self.ros_to_pb:
            raise ValueError('Joint {} is not the name of a joint'.format(joint))
        return p.getJointState(self.baxter_id, self.ros_to_pb[joint])[0]

    def joint_angles(self):
        joint_dict = dict()
        for key in self.pb_to_ros.keys():
            joint_dict[self.pb_to_ros[key]] = p.getJointState(self.baxter_id, key)[0]
        return joint_dict

    def joint_velocity(self, joint):
        if joint not in self.ros_to_pb:
            raise ValueError('Joint {} is not the name of a joint'.format(joint))
        return p.getJointState(self.baxter_id, self.ros_to_pb[joint])[1]

    def joint_velocities(self):
        joint_dict = dict()
        for key in self.pb_to_ros.keys():
            joint_dict[self.pb_to_ros[key]] = p.getJointState(self.baxter_id, key)[1]
        return joint_dict

    def joint_effort(self, joint):
        if joint not in self.ros_to_pb:
            raise ValueError('Joint {} is not the name of a joint'.format(joint))
        return p.getJointState(self.baxter_id, self.ros_to_pb[joint])[3]

    def joint_efforts(self):
        joint_dict = dict()
        for key in self.pb_to_ros.keys():
            joint_dict[self.pb_to_ros[key]] = p.getJointState(self.baxter_id, key)[3]
        return joint_dict

    def endpoint_pose(self):
        pos, orn = p.getLinkState(self.robot_id, self._ee_index)
        return {'orientation': list(orn), 'position': list(pos)}

    def endpoint_velocity(self):
        # return deepcopy(self._cartesian_velocity)
        return

    def endpoint_effort(self):
        # return deepcopy(self._cartesian_effort)
        return

    def set_joint_position_speed(self, speed):
        return

    def set_joint_positions(self, positions, raw=False):
        return

    def set_joint_velocities(self, joints_dict, control_type,
                                max_force, max_vel, pos_gain, vel_gain):
        """
        Args
            joints (dict)
        """
        joints, velocities = joints_dict.keys(), joints_dict.value()
        p.setJointMotorControlArray(bodyUniqueId=self.baxter_id,
                                    jointIndices=joints,
                                    controlMode=control_type,
                                    # targetPositions=angles,
                                    targetVelocity=velocities,
                                    force=max_force,
                                    maxVelocity=max_vel,
                                    positionGain=pos_gain,
                                    velocityGain=vel_gain)
        return

    def set_joint_torques(self, joints_dict, control_type,
                                max_force, max_vel, pos_gain, vel_gain):
        """
        Args
            joints (dict)
        """
        #TODO Fix this
        joints, torques = joints_dict.keys(), joints_dict.value()
        p.setJointMotorControlArray(bodyUniqueId=self.baxter_id,
                                    jointIndices=joints,
                                    controlMode=control_type,
                                    # targetPositions=angles,
                                    targetVelocity=0,
                                    force=max_force,
                                    maxVelocity=max_vel,
                                    positionGain=pos_gain,
                                    velocityGain=vel_gain)
        return

    def move_to_neutral(self):
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
        return self.move_to_joint_positions(angles, timeout)

    def move_to_joint_positions(self, joints_dict, control_type,
                                max_force, max_vel, pos_gain, vel_gain):
        """
        Args
            joints (dict)
        """
        joints, angles = joints_dict.keys(), joints_dict.value()
        p.setJointMotorControlArray(bodyUniqueId=self.baxter_id,
                                    jointIndices=joints,
                                    controlMode=control_type,
                                    targetPositions=angles,
                                    targetVelocity=0,
                                    force=max_force,
                                    maxVelocity=max_vel,
                                    positionGain=pos_gain,
                                    velocityGain=vel_gain)
        return

class Gripper(object):
    def __init__(self, robot_id, gripper):
        self.robot_id = robot_id

    def close(self):
        pass

    def open(self):
        pass
