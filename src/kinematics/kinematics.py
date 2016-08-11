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

        :param x: x-axis
        :param y: y-axis
        :param z: z-axis
        :return: all axis
        """

        result_set = [0, 0, 0, 0, 0]

        result_set[fixed_joint] = angle

        if fixed_joint == 0:
            """
            redundant because this is already checked in inverse() but this is kinda required to allow this function to be called alone
            """
            print("Base axis cannot be fixed joint!")
            return

        elif fixed_joint == 1:
            print("Fixed axis " + fixed_joint + " not yet implemented.")

        elif fixed_joint == 2:
            print("Fixed axis " + fixed_joint + " not yet implemented.")

        elif fixed_joint == 3:

            # Vektor to the given P
            pv = math.sqrt(r**2 + z**2)
            print("Pv: " + str(pv))

            # Calculate b
            thetab2 = math.pi - angle
            print("Theta_b2: " + str(thetab2))

            b = math.sqrt(self.links[1]**2 + self.links[2]**2 - 2*self.links[1]*self.links[2]*math.cos(thetab2))
            print("b: " + str(b))

            # calculate theta1
            thetab = math.acos((-b**2 + self.links[0]**2 + pv**2) / (2*self.links[0]*pv))
            print("Theta_b: " + str(thetab))


            thetaz = math.asin(r/pv)
            print("Theta_z: " + str(thetaz))

            theta1 = thetab + thetaz
            print("Theta_1: " + str(theta1))

            #Calculate theta2
            thetal2 = math.acos((-self.links[2]**2 + self.links[1]**2 + b**2) / (2*self.links[1]*b))
            print("Theta_l2: " + str(thetal2))

            thetaP = math.acos((-pv**2 + self.links[0]**2 + b**2) / (2*self.links[0]*b))
            print("Theta_P: " + str(thetaP))

            theta2 = math.pi - thetal2 - thetaP
            print("Theta_2: " + str(theta2))

            result_set[0] = phi
            result_set[1] = theta1
            result_set[2] = theta2
            result_set[3] = angle

        return result_set
