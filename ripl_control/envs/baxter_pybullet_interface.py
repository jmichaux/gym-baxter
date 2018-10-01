import numpy as np
import pybullet as p

class Limb(object):
    """
    PyBullet interface class that mimics the Limb class from baxter_interface.limb.
    """
    def __init__(self, baxter_id, name, config):
        self.baxter_id = baxter_id
        self.name = name
        self.config = config
        if self.name not in ['left', 'right']:
            raise ValueError("Arg 'name' must be 'right' or 'left'.")
        if self.name == 'left':
            self._joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                                'left_w0', 'left_w1', 'left_w2']
            self.joints = self.joint_indices = [34, 35, 36, 37, 38, 40, 41]
            self.ee = self.ee_index = 48
        else:
            self._joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                                'right_w0', 'right_w1', 'right_w2']
            self.joints = self.joint_indices = [12, 13, 14, 15, 16, 18, 19]
            self.ee = self.ee_index = 26
        # Real2Sim conversions
        self._name_2_ind = dict(zip(self._joint_names, self.joint_indices))
        self._ind_2_name = dict(zip(self.joint_indices, self._joint_names))

    def joint_names(self):
        return self._joint_names

    def joint_angle(self, joint):
        return p.getJointState(self.baxter_id, self.joint_indices[joint])[0]

    def joint_angles(self):
        joints = []
        for joint_index in self.joint_indices:
            joints.append(p.getJointState(self.baxter_id, joint_index)[0])
        return joints

    def joint_velocity(self, joint):
        return p.getJointState(self.baxter_id, self.joint_indices[joint])[1]

    def joint_velocities(self):
        joints = []
        for joint_index in self.joint_indices:
            joints.append(p.getJointState(self.baxter_id, joint_index)[1])
        return joints

    def joint_effort(self, joint):
        return p.getJointState(self.baxter_id, self.joint_indices[joint])[3]

    def joint_efforts(self):
        joints = []
        for joint_index in self.joint_indices:
            joints.append(p.getJointState(self.baxter_id, joint_index)[3])
        return joints

    def endpoint_pose(self):
        pos, orn = p.getLinkState(self.baxter_id, self.ee_index)[:2]
        return {'position': pos, 'orientation': orn}


    def endpoint_velocity(self):
        """
        Cartesian velocity
        """
        return

    def endpoint_effort(self):
        """
        Cartesian effort
        """
        return

    def set_joint_position_speed(self, speed):
        # This might not  be necessary
        return

    def set_joint_positions(self, joints):
        """
        This is kinda silly, but it matches the syntax in baxter.py
        in order to execute control commands for both arms simultaneously.
        """
        self.move_to_joint_positions(joints)
        return

    # def set_joint_velocities(self,
    #                          joints_velocities,
    #                          target_positions=self.config.target_positions,
    #                          max_force=self.config.max_force,
    #                          max_velocity=self.config.max_velocity,
    #                          position_gain=self.config.position_gain,
    #                          velocity_gain=self.config.velocity_gain):
    #
    #     """
    #     Args
    #         joints (list)
    #     """
    #     joint_indices, velocities = joints_dict.keys(), joints_dict.value()
    #     p.setJointMotorControlArray(bodyUniqueId=self.baxter_id,
    #                                 jointIndices=self.joint_indices,
    #                                 controlMode=p.VELOCITY_CONTROL,
    #                                 targetPositions=target_positions,
    #                                 targetVelocity=joint_velocities,
    #                                 force=max_force,
    #                                 maxVelocity=max_velocity,
    #                                 positionGain=position_gain,
    #                                 velocityGain=velocity_gain)
    #     return
    #
    # def set_joint_torques(self,
    #                       joint_torques,
    #                       target_positions=self.config.target_positions,
    #                       target_velocity=self.config.target_velocity,
    #                       max_force=self.config.max_force,
    #                       max_velocity=self.config.max_velocity,
    #                       position_gain=self.config.position_gain,
    #                       velocity_gain=self.config.velocity_gain):
    #
    #
    #     """
    #     Args
    #         joints (list): List of joint position values
    #     """
    #     p.setJointMotorControlArray(bodyUniqueId=self.baxter_id,
    #                                 jointIndices=self.joint_indices,
    #                                 controlMode=p.TORQUE_CONTROL,
    #                                 targetPositions=target_positions,
    #                                 targetVelocity=target_velocity,
    #                                 force=joint_torques,
    #                                 maxVelocity=max_velocity,
    #                                 positionGain=position_gain,
    #                                 velocityGain=velocity_gain)
    #     return

    def move_to_neutral(self):
        initial_pose = self.config.initial_pose
        for joint_index, joint_val in initial_pose:
            p.resetJointState(self.baxter_id, joint_index, joint_val)
            p.setJointMotorControl2(baxter, joint_index, p.POSITION_CONTROL, joint_val, force=self.max_force)

    def move_to_joint_positions(self, joint_angles):
        joint_indices=self.joint_indices
        target_velocity=self.config.target_velocity
        max_force=self.config.max_force
        max_velocity=self.config.max_velocity
        position_gain=self.config.position_gain
        velocity_gain=self.config.velocity_gain
        joint_indices = [5, 12, 13, 14, 15, 16, 18, 19, 27, 29, 34, 35, 36, 37, 38, 40, 41, 49, 51]
        """
        Args
            joints (list): List of joint position values
        """
        p.setJointMotorControlArray(bodyIndex=self.baxter_id,
                                    jointIndices=joint_indices,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=joint_angles)
                                    # targetVelocities=target_velocity,
                                    # forces=max_force,
                                    # positionGains=position_gain,
                                    # velocityGains=velocity_gain)
        return

class Gripper(object):
    def __init__(self, baxter_id, gripper):
        self.baxter_id = baxter_id

    def close(self):
        pass

    def open(self):
        pass
