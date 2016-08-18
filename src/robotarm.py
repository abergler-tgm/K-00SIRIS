import math

from kinematics import Kinematics


class Robotarm:
    """
    Main controller-class for the 00SIRIS Robotarm
    """

    # TODO: Integrate Hedgehog and get this going
    """
    Hedgehog information:
    2000 Steps per Servo
    """

    def __init__(self, links, base_offset, min_angles, max_angles, min_servo_pos, max_servo_pos, fixed_joint=3, fj_angle=0):
        """
        Constructor - used for initialization

        :param links: the lengths of all links in an array
        :param base_offset: the distance in between the center of the base axis and the first joint
        """

        # Params
        self.links = links
        self.base_offset = base_offset
        self.min_angles = min_angles
        self.max_angles = max_angles
        self.min_servo_pos = min_servo_pos
        self.max_servo_pos = max_servo_pos
        self.fixed_joint = fixed_joint
        self.fj_angle = fj_angle

        self.kinematics = Kinematics(self.links, self.base_offset)

    def move_to_angle(self, angle, joint):
        """
        Moves a joint to a certain angle
        :param angle: the angle
        :param joint: the joint
        """
        if self.min_angles[joint] < angle < self.max_angles[joint]:
            pass    # TODO: raise error

        total_steps = (self.max_servo_pos[joint]-self.min_servo_pos[joint])

        pos = (total_steps/(2*math.pi)) + self.min_servo_pos[joint]

        # TODO test properly

        return pos

    def move_to_cartesian(self, x, y, z):
        """
        Moves the robotarm to the given cartesian coordinates.

        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        """

        angles = self.kinematics.inverse(x, y, z, self.fixed_joint, self.fj_angle)

        # TODO

        # exception handling
        # validate
        # move servos to angles

    def move_direct(self, angles):
        """
        Move the robotarm to the given configuration.

        :param angles: the angles of all joints
        """

        # TODO

        # validate
        # move servos to angles
        pass

    def validate_configuration(self, angles):
        """
        Validates whether the given configuration is reachable.

        :param angles: the angles of all joints
        :return: True if the given configuration is reachable, else: False
        """

        if angles != 5:
            return False

        # Check whether the angles on all joints can be reached
        for i, angle in enumerate(angles):
            if not self.validate_angle(angle, i):
                return False

        return True

    def validate_angle(self, angle, joint):
        """
        Validates whether the given angle is reachable with the given joint.

        :param angle: the angle that should be validated
        :param joint: the joint that the angle should be validated for
        :return: True if the given angle is reachable with the given joint, else: False
        """
        return self.min_angles[joint] < angle < self.max_angles[joint]

class Joint:

    def __init__(self, ):