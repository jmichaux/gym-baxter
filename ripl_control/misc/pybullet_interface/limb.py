import numpy as np
import pybullet as p


class Limb(object):
    def __init__(self, robot_id, joint_dict):
        self.robot_id = robot_id

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.
        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names[self.name]

    def joint_angle(self, joint):
        """
        Return the requested joint angle.
        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self._joint_angle[joint]

    def joint_angles(self):
        """
        Return all joint angles.
        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return deepcopy(self._joint_angle)

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.
        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self._joint_velocity[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.
        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return deepcopy(self._joint_velocity)

    def joint_effort(self, joint):
        """
        Return the requested joint effort.
        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: effort in Nm of individual joint
        """
        return self._joint_effort[joint]

    def joint_efforts(self):
        """
        Return all joint efforts.
        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to effort (Nm) Values
        """
        return deepcopy(self._joint_effort)

    def endpoint_pose(self):
        """
        Return Cartesian endpoint pose {position, orientation}.
        @rtype: dict({str:L{Limb.Point},str:L{Limb.Quaternion}})
        @return: position and orientation as named tuples in a dict
        C{pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}}
          - 'position': Cartesian coordinates x,y,z in
                        namedtuple L{Limb.Point}
          - 'orientation': quaternion x,y,z,w in named tuple
                           L{Limb.Quaternion}
        """
        return deepcopy(self._cartesian_pose)

    def endpoint_velocity(self):
        """
        Return Cartesian endpoint twist {linear, angular}.
        @rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        @return: linear and angular velocities as named tuples in a dict
        C{twist = {'linear': (x, y, z), 'angular': (x, y, z)}}
          - 'linear': Cartesian velocity in x,y,z directions in
                      namedtuple L{Limb.Point}
          - 'angular': Angular velocity around x,y,z axes in named tuple
                       L{Limb.Point}
        """
        return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self):
        """
        Return Cartesian endpoint wrench {force, torque}.
        @rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        @return: force and torque at endpoint as named tuples in a dict
        C{wrench = {'force': (x, y, z), 'torque': (x, y, z)}}
          - 'force': Cartesian force on x,y,z axes in
                     namedtuple L{Limb.Point}
          - 'torque': Torque around x,y,z axes in named tuple
                      L{Limb.Point}
        """
        return deepcopy(self._cartesian_effort)

    def set_command_timeout(self, timeout):
        """
        Set the timeout in seconds for the joint controller
        @type timeout: float
        @param timeout: timeout in seconds
        """
        self._pub_joint_cmd_timeout.publish(Float64(timeout))

    def exit_control_mode(self, timeout=0.2):
        """
        Clean exit from advanced control modes (joint torque or velocity).
        Resets control to joint position mode with current positions.
        @type timeout: float
        @param timeout: control timeout in seconds [0.2]
        """
        self.set_command_timeout(timeout)
        self.set_joint_positions(self.joint_angles())

    def set_joint_position_speed(self, speed):
        """
        Set ratio of max joint speed to use during joint position moves.
        Set the proportion of maximum controllable velocity to use
        during joint position control execution. The default ratio
        is `0.3`, and can be set anywhere from [0.0-1.0] (clipped).
        Once set, a speed ratio will persist until a new execution
        speed is set.
        @type speed: float
        @param speed: ratio of maximum joint speed for execution
                      default= 0.3; range= [0.0-1.0]
        """
        self._pub_speed_ratio.publish(Float64(speed))

    def set_joint_positions(self, positions, raw=False):
        """
        Commands the joints of this limb to the specified positions.
        B{IMPORTANT:} 'raw' joint position control mode allows for commanding
        joint positions, without modification, directly to the JCBs
        (Joint Controller Boards). While this results in more unaffected
        motions, 'raw' joint position control mode bypasses the safety system
        modifications (e.g. collision avoidance).
        Please use with caution.
        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type raw: bool
        @param raw: advanced, direct position control mode
        """
        self._command_msg.names = positions.keys()
        self._command_msg.command = positions.values()
        if raw:
            self._command_msg.mode = JointCommand.RAW_POSITION_MODE
        else:
            self._command_msg.mode = JointCommand.POSITION_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_velocities(self, velocities):
        """
        Commands the joints of this limb to the specified velocities.
        B{IMPORTANT}: set_joint_velocities must be commanded at a rate great
        than the timeout specified by set_command_timeout. If the timeout is
        exceeded before a new set_joint_velocities command is received, the
        controller will switch modes back to position mode for safety purposes.
        @type velocities: dict({str:float})
        @param velocities: joint_name:velocity command
        """
        self._command_msg.names = velocities.keys()
        self._command_msg.command = velocities.values()
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_torques(self, torques):
        """
        Commands the joints of this limb to the specified torques.
        B{IMPORTANT}: set_joint_torques must be commanded at a rate great than
        the timeout specified by set_command_timeout. If the timeout is
        exceeded before a new set_joint_torques command is received, the
        controller will switch modes back to position mode for safety purposes.
        @type torques: dict({str:float})
        @param torques: joint_name:torque command
        """
        self._command_msg.names = torques.keys()
        self._command_msg.command = torques.values()
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def move_to_neutral(self):
        """
        Command the joints to the center of their joint ranges
        Neutral is defined as::
          ['*_s0', '*_s1', '*_e0', '*_e1', '*_w0', '*_w1', '*_w2']
          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        """
        angles = dict(zip(self.joint_names(),
                          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        return self.move_to_joint_positions(angles, timeout)

    def move_to_joint_positions(self, positions):
        """
        (Blocking) Commands the limb to the provided positions.
        Waits until the reported joint state matches that specified.
        This function uses a low-pass filter to smooth the movement.
        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        """
        for joint_index, joint_val in joint_poses:
            p.setJointMotorControl2(self.baxter_id, joint_index, p.POSITION_CONTROL, joint_val)
