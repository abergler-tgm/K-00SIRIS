import math


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
        x = 0
        y = 0
        z = 0

        return x, y, z

    def direct_cylindric(self, angles):
        """
        This calculates the cylindrical coordinates of the TCP using the given angles

        :param angles: angles of all joints
        :return: the end position
        """

        # TODO
        x = 0
        y = 0
        z = 0

        return x, y, z

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

            # TODO use exceptions

            return

        print("x: " + str(x) + " y: " + str(y) + " z: " + str(z))

        r = math.sqrt(x**2 + y**2)
        print("r: " + str(r))

        phi = math.atan(y/x)
        print("phi: " + str(phi))

        # #later:
        return self.inverse_cylindric(r, z, phi, fixed_joint, angle)

    def inverse_cylindric(self, r, z, phi, fixed_joint, angle):
        """
        This calculates all remaining axis angles for the given r, z, phi cylindrical coordinate and fixed joint

        :param r: the radius
        :param z: the z-coordinate
        :param phi: the angle of the coordinate (phi)
        :param fixed_joint: the index of the fixed joint
        :param angle: the angle of the fixed joint
        :return: all axis inverse kinematics result
        """

        result_set = [0, 0, 0, 0]

        result_set[fixed_joint] = angle

        # Vector to the given P
        pv = math.sqrt(r ** 2 + z ** 2)
        print("Pv: " + str(pv))

        if fixed_joint == 0:
            """
            redundant because this is already checked in inverse() but this is kinda required to
            allow this function to be called alone
            """
            print("Base axis cannot be fixed joint!")
            # TODO use exceptions
            return

        elif fixed_joint == 1:
            # TODO: Somewhere in this block is a bug that causes wrong calculations.
            """
            Not sure whether the bug is in the code itself or it is a math problem (wrong formulas)
            """

            theta_1 = angle

            # Calculate b
            theta_z = math.asin(r/pv)
            theta_b = theta_1 - theta_z

            b = math.sqrt(self.links[0]**2 + pv**2 - 2*self.links[0]*pv*math.cos(theta_b))
            print("b: " + str(b))

            # Calculate theta_2
            try:
                theta_l2 = math.acos((-self.links[2] ** 2 + self.links[1] ** 2 + b ** 2) / (2 * self.links[1] * b))
                print("Theta_l2: " + str(theta_l2))
            except ValueError as e:
                print("Error while calculating theta_l2!")
                # TODO use exceptions
                return

            try:
                theta_pv = math.acos((-pv ** 2 + self.links[0] ** 2 + b ** 2) / (2 * self.links[0] * b))
                print("Theta_pv: " + str(theta_pv))
            except ValueError as e:
                print("Error while calculating theta_pv!")
                # TODO use exceptions
                return

            theta_2 = math.pi - theta_l2 - theta_pv
            print("Theta_2: " + str(theta_2))

            # Calculate theta_3
            try:
                theta_b2 = math.acos((-b**2 + self.links[1]**2 + self.links[2]**2) / (2*self.links[1]*self.links[2]))
                print("Theta_b2: " + str(theta_pv))
            except ValueError as e:
                print("Error while calculating theta_pv!")
                # TODO use exceptions
                return

            theta_3 = math.pi - theta_b2
            print("Theta_3: " + str(theta_3))

            # Build result_set
            result_set[0] = phi
            result_set[1] = theta_1
            result_set[2] = theta_2
            result_set[3] = theta_3


        elif fixed_joint == 2:
            print("Fixed axis " + str(fixed_joint) + " not yet implemented.")
            # TODO implement this

        elif fixed_joint == 3:
            theta_3 = angle

            # Calculate b
            theta_b2 = math.pi - theta_3
            print("Theta_b2: " + str(theta_b2))

            b = math.sqrt(self.links[1]**2 + self.links[2]**2 - 2*self.links[1]*self.links[2]*math.cos(theta_b2))
            print("b: " + str(b))

            # calculate theta_1
            try:
                theta_b = math.acos((-b**2 + self.links[0]**2 + pv**2) / (2*self.links[0]*pv))
                print("Theta_b: " + str(theta_b))
            except ValueError as e:
                print("Error while calculating theta_b!")
                # TODO use exceptions
                return

            try:
                theta_z = math.asin(r/pv)
                print("Theta_z: " + str(theta_z))
            except ValueError as e:
                print("Error while calculating theta_z!")
                # TODO use exceptions
                return

            theta_1 = theta_b + theta_z
            print("Theta_1: " + str(theta_1))

            # Calculate theta_2
            try:
                theta_l2 = math.acos((-self.links[2]**2 + self.links[1]**2 + b**2) / (2*self.links[1]*b))
                print("Theta_l2: " + str(theta_l2))
            except ValueError as e:
                print("Error while calculating theta_l2!")
                # TODO use exceptions
                return

            try:
                theta_pv = math.acos((-pv**2 + self.links[0]**2 + b**2) / (2*self.links[0]*b))
                print("Theta_pv: " + str(theta_pv))
            except ValueError as e:
                print("Error while calculating theta_pv!")
                # TODO use exceptions
                return

            theta_2 = math.pi - theta_l2 - theta_pv
            print("Theta_2: " + str(theta_2))

            # Build result_set
            result_set[0] = phi
            result_set[1] = theta_1
            result_set[2] = theta_2
            result_set[3] = theta_3

        return result_set

    def validate_kinematics(self, x, y, z, angles):
        """
        Validates whether the given points can be reached
        :param x:
        :param y:
        :param z:
        :param angles:
        :return:
        """

        # TODO

        check_x, check_y, check_z = self.direct(angles)

        return x == check_x and y == check_y and z == check_z

    @staticmethod
    def round_results(results):
        """
        Rounds the given result-set up or down to 2 digits
        :param results: the result-set that should be rounded
        :return: the rounded result-set
        """

        rounded = []
        for result in results:
            rounded.append(int((result * 100) + 0.5) / 100.0)

        return rounded
