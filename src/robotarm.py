import math
import time

from kinematics import Kinematics, KinematicsError, WorldCoordinateSystem


class Robotarm:
    """
    Main controller-class for the 00SIRIS Robotarm
        Hedgehog information:
            -2000 Steps per Servo
    """

    def __init__(self, client, links, base_offset, min_angles, max_angles, min_servo_pos, max_servo_pos):
        """
        Constructor - used for initialization

        :param links: the lengths of all links in an array
        :param base_offset: the distance in between the center of the base axis and the first joint
        """

        # Params
        self.client = client
        self.links = links
        self.base_offset = base_offset
        self.min_angles = min_angles
        self.max_angles = max_angles
        self.min_servo_pos = min_servo_pos
        self.max_servo_pos = max_servo_pos

        # Move to initial position
        # Base, axis 1, axis 2, axis 3, axis 4, grabber
        self.joint_pos = [0.0, 1.51, -1.51, 0.0, 0.0, 0.0]
        self.move_to_init()

        self.kinematics = Kinematics(self.links, self.base_offset)

    def move_to_init(self):
        """
        Moves the robotarm to the initial position
        """
        print("Moving to init!")
        self.move_to_config([0.0, 1.51, -1.51, 0.0, 0.0, 0.0])

    def move_to_pos(self, pos, joint):
        """
        Moves the given joint to the given position
        :param pos: the position
        :param joint: the joint that will be used
        :raise OutOfReachError: if the given position cannot be reached
        """
        if 0 <= pos <= 2000:    # TODO: maybe this can be changed to servo_min/max but this has to be checked properly
            self.client.set_servo(joint, True, int(pos))
            self.joint_pos[joint] = pos
        else:
            raise OutOfReachError("Position out of reach! (Range 0 to 2000): " + str(pos))

    def move_to_angle(self, angle, joint):
        """
        Moves a joint to a certain angle
        :param angle: the angle
        :param joint: the joint
        """
        pos = self.angle_to_step(angle, joint)

        self.move_to_pos(pos, joint)

    def angle_to_step(self, angle, joint):
        """
        Calculates the step value of the given angle for the given joint.
        Servos have 2048 steps and this maps the angle to those
        :param angle: the angle in radiant representation
        :param joint: the joint that should be used
        :return: the servo-position in steps
        """
        if not self.validate_angle(angle, joint):
            raise OutOfReachError

        if self.min_angles[joint] == self.max_angles[joint]:
            return 0    # if no min/max angle is set for the servo, position does not matter

        total_steps = (self.max_servo_pos[joint] - self.min_servo_pos[joint])
        total_angle = abs(self.min_angles[joint]) + abs(self.max_angles[joint])

        print("ANGLE to STEP: Total steps: " + str(total_steps) + " Total angle: " + str(total_angle))

        angle += abs(self.min_angles[joint])
        print("Angle after correction: " + str(angle))

        # TODO: Maybe proper rounding is worth it
        pos = int((total_steps / total_angle) * angle + self.min_servo_pos[joint])

        print("Calculated pos: " + str(pos))
        # TODO: Test properly
        # TODO: Probably very buggy

        return pos

    def step_to_angle(self, pos, joint):
        """
        Calculates the angle of the given servo-position (in steps) for the given joint.
        Servos have 2048 steps and this maps the angle to those
        :param pos: the given step
        :param joint: the joint that should be used
        :return: the servo-position in angle
        """

        # TODO: Reimplement this!

        return 0

    def move_to_cartesian(self, x, y, z, fixed_joint, fj_angle):
        """
        Moves the robotarm to the given cartesian coordinates.
        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :param fixed_joint:
        :param fj_angle:
        """

        try:
            angles = self.kinematics.inverse(x, y, z, fixed_joint, fj_angle)
            self.move_to_config(angles)
        except KinematicsError as ke:
            print("Position cannot be reached: " + ke.message)

        except OutOfReachError as oore:
            print("The kinematics module was able to calculate a position, but the servos are not able to reach it: " +
                  oore.message)

    def move_to_cartesian_aligned(self, x, y, z, alignment):
        """
        Moves the robotarm to the given cartesian coordinates.
        This one takes the alignment of the grabber towards the x-axis and uses it as the fixed joint.
        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        """

        try:
            angles = self.kinematics.inverse_aligned(x, y, z, alignment)
            self.move_to_config(angles)
        except KinematicsError as ke:
            print("Position cannot be reached: " + ke.message)

        except OutOfReachError as oore:
            print("The kinematics module was able to calculate a position, but the servos are not able to reach it: " +
                  oore.message)

    def move_to_config(self, angles):
        """
        Move the robotarm to the given configuration.
        :param angles: the angles of all joints
        """

        if self.validate_configuration(angles):
            cfg = [(joint, True, self.angle_to_step(angle, joint)) for joint, angle in enumerate(angles)]
            print("Moving to:")
            # TODO: Just using the first 4 axis for now([:4]). Needs a 2nd hedgehog for more
            print(cfg[:4])
            self.client.set_multi_servo(cfg[:4])
            print("Movement started, should be done very soon!")
        else:
            raise OutOfReachError("The given configuration cannot be reached!")

    def validate_configuration(self, angles):
        """
        Validates whether the given configuration is reachable.
        :param angles: the angles of all joints
        :return: True if the given configuration is reachable, else: False
        """
        # TODO test properly

        if len(angles) < 4:
            return False  # the K-00SIRIS needs at least 4 joints for a kinematic configuration

        # Check whether the angles on all joints can be reached
        for joint, angle in enumerate(angles):
            if not self.validate_angle(angle, joint):
                return False

        return True

    def validate_angle(self, angle, joint):
        """
        Validates whether the given angle is reachable with the given joint.
        :param angle: the angle that should be validated
        :param joint: the joint that the angle should be validated for
        :return: True if the given angle is reachable with the given joint, else: False
        """

        print("Validating: Joint: " + str(joint) + " Angle: " + str(angle))
        if self.min_angles[joint] <= angle <= self.max_angles[joint] or self.min_angles[joint] >= angle >= self.max_angles[joint]:
            print("Angle is valid!")
            return True
        else:
            print("Angle is invalid!")
            return False

    def move_servo_over_time(self, pos, joint, duration):
        """
        Moves a joint to the given position in exactly the given duration
        :param pos: the position that the servo should move to
        :param joint: the joint that should be moved
        :param duration: the duration that the movement should take
        """
        start_position = self.joint_pos[joint]
        movement = pos - start_position

        print("Starting!")

        if movement == 0:
            print("Already at position")
            return  # Already at position

        print("Starting procedure!")

        time_per_step = abs(duration / movement)
        start_time = time.time()

        while start_time + duration > time.time() and pos != self.joint_pos[joint]:
            crnt_step = round((time.time() - start_time) / time_per_step)
            self.move_to_pos(start_position + crnt_step * math.copysign(1, movement), joint)
            time.sleep(0.05)  # To prevent from overload

        """
        Not very beautiful, but this will ensure that the pos will definitely be reached even
        if there is an issue with the duration/loop
        """
        self.move_to_pos(pos, joint)

    def get_tool_cs(self):
        """
        Returns the tool coordinate system
        :return: the tool coordinate system as WorldCoordinateSystem
        """
        angles = []
        for joint, pos in enumerate(self.joint_pos):
            angles[joint] = self.step_to_angle(pos, joint)

        x, y, z, theta_x, theta_y, theta_z = self.kinematics.direct(angles)
        return WorldCoordinateSystem(x, y, z, theta_x, theta_y, theta_z)

    def servos_off(self):
        """
        Turns off all servos
        """
        self.client.set_multi_servo([(0, False, 0), (1, False, 0), (2, False, 0), (3, False, 0)])


class RobotarmError(Exception):
    """
    Is thrown when an error occurs within the Robotarm module
    """

    def __init__(self, message):
        """
        Initializes the exception
        :param message: explanation of the error
        """

        self.message = message


class OutOfReachError(RobotarmError):
    """
    Is thrown when a point or position is out of reach
    """
    pass
