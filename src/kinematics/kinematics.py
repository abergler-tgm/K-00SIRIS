from math import sqrt, atan


class Kinematics:
    """
    This is the revised kinematics module used for 00SIRIS
    """

    def __init__(self, links, base_offset):
        """
        Constructor - used for initialization
        
        :param links: the lengths of all links in an array
        :param base_offset: the distance in between the center of the base axis and the first axis
        """

        self.links = links
        self.base_offset = base_offset

    def direct(self, angles):
        """
        This calculates cartesian coordinates of the TCP using the given angles

        :param angles: angles of all joints
        :return: the end position
        """

        # TODO
        pass

    def direct_cylindric(self, angles):
        """
        This calculates the cylindrical coordinates of the TCP using the given angles

        :param angles: angles of all joints
        :return: the end position
        """

        # TODO
        pass

    def inverse(self, x, y, z, fixed_joint, angle):
        """
        This calculates all remaining axis angles for the given x, y, z coordinate and fixed joint
        One of the angles has to be predefined -> This one is the fixed_joint
        Base axis cannot be the fixed one!

        :param x: x-axis
        :param y: y-axis
        :param z: z-axis
        :param fixed_joint: the index of the fixed joint
        :param angle: the angle of the fixed joint
        :return: all axis inverse kinematics result
        """

        if fixed_joint == 0:
            print("Base axis cannot be fixed joint!")
            return

        r = sqrt(x^2 + y^2)
        print("r: " + r)

        phi = atan(y/x)
        print("phi: " + phi)

        return self.inverse_cylindric(r, z, phi, fixed_joint, angle)

    def inverse_cylindric(self, r, z, phi, fixed_joint, angle):
        """
        This calculates all remaining axis angles for the given r, z, phi cylindrical coordinate and fixed joint

        :param x: x-axis
        :param y: y-axis
        :param z: z-axis
        :return: all axis
        """

        resultset = [0, 0, 0, 0, 0]

        resultset[fixed_joint] = angle

        # TODO

        return resultset
