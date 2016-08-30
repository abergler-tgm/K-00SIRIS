import math

import numpy as np


class Kinematics:
    """
    This is the revised kinematics module used for 00SIRIS
    """

    def __init__(self, links, base_offset):
        """
        Constructor - used for initialization
        
        :param links: the lengths of all links in an array
        :param base_offset: the distance in between the center of the base axis and the first joint
        """

        self.links = links
        self.base_offset = base_offset

        # Dictionary for storing world coordinate systems
        self.world_cs = {}

    def direct(self, angles):
        """
        This calculates cartesian coordinates of the TCP using the given angles

        :param angles: angles of all joints
        :return: the end position (x,y,z) and the grabber align (x_theta, y_theta, z_theta)
        """
        # TODO
        x = 0
        y = 0
        z = 0
        x_theta = 0
        y_theta = 0
        z_theta = 0

        return x, y, z, x_theta, y_theta, z_theta

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
        This calculates all remaining joint angles for the given x, y, z coordinate and fixed joint
        One of the angles has to be predefined -> This one is the fixed_joint
        Base axis cannot be the fixed one!

        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :param fixed_joint: the index of the fixed joint
        :param angle: the angle of the fixed joint
        :return: all joints inverse kinematics result
        """

        if fixed_joint == 0:
            """
            If base axis is fixed, the robot arm cannot turn...
            """
            raise ParameterError("Base axis cannot be fixed joint!")

        print("x: " + str(x) + " y: " + str(y) + " z: " + str(z))

        r, z, phi = self.cartesian_to_cylindric(x, y, z)

        return self.inverse_cylindric(r, z, phi, fixed_joint, angle)

    def inverse_cylindric(self, r, z, phi, fixed_joint, angle):
        """
        This calculates all remaining joint angles for the given r, z, phi cylindrical coordinate and fixed joint

        :param r: the radius
        :param z: the z-coordinate
        :param phi: the angle of the coordinate (phi)
        :param fixed_joint: the index of the fixed joint
        :param angle: the angle of the fixed joint
        :return: all joints inverse kinematics result
        """

        print("r: " + str(r) + "z: " + str(z) + "phi: " + str(phi))

        # Move coordinate system to first joint "root" to remove the offset
        r = r - self.base_offset

        result_set = [0, 0, 0, 0]

        # Vector to the given P
        vp = math.sqrt(r ** 2 + z ** 2)
        print("Pv: " + str(vp))

        if fixed_joint == 0:
            raise ParameterError("Base axis cannot be fixed joint!")

        elif fixed_joint == 1:
            # TODO: Write more test-cases

            theta_1 = angle

            # Calculate b
            theta_tcp = math.asin(z / vp)
            theta_b = theta_1 - theta_tcp

            b = math.sqrt(self.links[0] ** 2 + vp ** 2 - 2 * self.links[0] * vp * math.cos(theta_b))
            print("b: " + str(b))

            # Calculate theta_2
            try:
                theta_l2 = math.acos((-self.links[2] ** 2 + self.links[1] ** 2 + b ** 2) / (2 * self.links[1] * b))
                print("Theta_l2: " + str(theta_l2))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_l2! - The point can most likely not be reached.")

            try:
                theta_vp = math.acos((-vp ** 2 + self.links[0] ** 2 + b ** 2) / (2 * self.links[0] * b))
                print("Theta_vp: " + str(theta_vp))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_vp! - The point can most likely not be reached.")

            theta_2 = -(math.pi - theta_l2 - theta_vp)
            print("Theta_2: " + str(theta_2))

            # Calculate theta_3
            try:
                theta_b2 = math.acos(
                    (-b ** 2 + self.links[1] ** 2 + self.links[2] ** 2) / (2 * self.links[1] * self.links[2]))
                print("Theta_b2: " + str(theta_vp))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_b2! - The point can most likely not be reached.")

            theta_3 = -(math.pi - theta_b2)
            print("Theta_3: " + str(theta_3))

        elif fixed_joint == 2:
            print("Fixed joint " + str(fixed_joint) + " not yet implemented.")
            # TODO implement this

        elif fixed_joint == 3:
            theta_3 = angle

            # Calculate b
            theta_b2 = math.pi + theta_3
            print("Theta_b2: " + str(theta_b2))

            b = math.sqrt(
                self.links[1] ** 2 + self.links[2] ** 2 - 2 * self.links[1] * self.links[2] * math.cos(theta_b2))
            print("b: " + str(b))

            # calculate theta_1
            try:
                theta_b = math.acos((-b ** 2 + self.links[0] ** 2 + vp ** 2) / (2 * self.links[0] * vp))
                print("Theta_b: " + str(theta_b))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_b! - The point can most likely not be reached.")

            try:
                theta_tcp = math.asin(z / vp)
                print("Theta_tcp: " + str(theta_tcp))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_tcp! - The point can most likely not be reached.")

            theta_1 = theta_b + theta_tcp
            print("Theta_1: " + str(theta_1))

            # Calculate theta_2
            try:
                theta_l2 = math.acos((-self.links[2] ** 2 + self.links[1] ** 2 + b ** 2) / (2 * self.links[1] * b)) \
                           * math.copysign(1, -theta_3)
                print("Theta_l2: " + str(theta_l2))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_l2! - The point can most likely not be reached.")

            try:
                theta_vp = math.acos((-vp ** 2 + self.links[0] ** 2 + b ** 2) / (2 * self.links[0] * b))
                print("Theta_vp: " + str(theta_vp))
            except ValueError as e:
                raise CalculationError("Error while calculating theta_vp! - The point can most likely not be reached.")

            theta_2 = -(math.pi - theta_l2 - theta_vp)
            print("Theta_2: " + str(theta_2))

        elif fixed_joint == 4:
            raise ParameterError("The 4th joint is used to turn the gripper and has no influence on the calculation.")

        elif fixed_joint > 4:
            raise ParameterError("The robot arm only has 5 degrees of freedom.")

        else:
            raise ParameterError("The fixed_joint parameter input is invalid.")

        # TODO: theta_1, theta_2 and theta_3 might be referenced before assignment (=dirty)
        # Build result_set
        result_set[0] = phi
        result_set[1] = theta_1
        result_set[2] = theta_2
        result_set[3] = theta_3

        return result_set

    def inverse_aligned(self, x, y, z, alignment):
        """
        This calculates all remaining joint angles for the given x, y, z coordinate and grabber alignment
        This one takes the alignment of the grabber towards the x-axis and uses it as the fixed joint.

        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :param alignment: the alignment of the grabber towards the x-axis
        :return: all joints inverse kinematics result
        """

        print("x: " + str(x) + " y: " + str(y) + " z: " + str(z))

        r, z, phi = self.cartesian_to_cylindric(x, y, z)

        return self.inverse_aligned_cylindric(r, z, phi, alignment)

    def inverse_aligned_cylindric(self, r, z, phi, alignment):
        """
        This calculates all remaining joint angles for the given r, z, phi coordinate and grabber alignment.
        This one takes the alignment of the grabber towards the x-axis and uses it as the fixed joint.

        :param r: the radius
        :param z: the z-coordinate
        :param phi: the angle of the coordinate (phi)
        :param alignment: the alignment of the grabber towards the x-axis
        :return: all joints inverse kinematics result
        """
        # TODO: Write test-cases

        print("r: " + str(r) + "z: " + str(z) + "phi: " + str(phi))

        # Move coordinate system to first joint "root" to remove the offset
        r = r - self.base_offset
        print("new r: " + str(r))

        result_set = [0, 0, 0, 0]

        # Vector to the given P
        vp = math.sqrt(r ** 2 + z ** 2)
        print("Pv: " + str(vp))

        # Calculate the position of the third joint (R3)
        r3_r = r - (math.cos(alignment) * self.links[2])
        r3_z = z - (math.sin(alignment) * self.links[2])
        print("Point R3: " + str(r3_r) + "/" + str(r3_z))

        # Vector to R3
        vr3 = math.sqrt(r3_r ** 2 + r3_z ** 2)
        print("Vector to R3: " + str(vr3))

        # Calculate the angle of the vector to the third joint (OVR3)
        try:
            theta_r3 = math.asin(r3_z / vr3)
            print("Theta_r3: " + str(theta_r3))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_r3! - The point can most likely not be reached.")

        # Calculate the triangle(R1, R2, R3)
        try:
            theta_vr3 = math.acos((-vr3**2 + self.links[0]**2 + self.links[1]**2) / (2 * self.links[0] * self.links[1]))
            print("Theta_vr3: " + str(theta_vr3))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_vr3! - The point can most likely not be reached.")

        try:
            theta_l0 = math.acos((-self.links[0]**2 + self.links[1]**2 + vr3**2) / (2 * self.links[1] * vr3))
            print("Theta_l0: " + str(theta_l0))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_l0! - The point can most likely not be reached.")

        try:
            theta_l1 = math.pi - theta_l0 - theta_vr3
            print("Theta_l1: " + str(theta_l1))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_l1! - The point can most likely not be reached.")

        # Calculate the angle to the given point
        try:
            theta_tcp = math.asin(z / vp)
            print("Theta_tcp: " + str(theta_tcp))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_tcp! - The point can most likely not be reached.")

        # Calculate 2 angles of the triangle(P0, R3, TCP)
        try:
            theta_vtcp = math.acos((-vp**2 + vr3**2 + self.links[2]**2) / (2 * vr3 * self.links[2]))
            print("Theta_vtcp: " + str(theta_vtcp))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_vtcp! - The point can most likely not be reached.")

        try:
            theta_l2 = math.acos((-self.links[2]**2 + vp**2 + vr3**2) / (2 * vp * vr3))
            print("Theta_l2: " + str(theta_l2))
        except ValueError as e:
            raise CalculationError("Error while calculating theta_l2! - The point can most likely not be reached.")

        # Calculate joint angles
        theta_1 = theta_tcp + theta_l2 + theta_l1
        print("Theta_1: " + str(theta_1))

        theta_2 = -(math.pi - theta_vr3)
        print("Theta_2: " + str(theta_2))

        theta_3 = -(math.pi - (theta_l0 + theta_vtcp))
        print("Theta_3: " + str(theta_3))

        # Build result_set
        result_set[0] = phi
        result_set[1] = theta_1
        result_set[2] = theta_2
        result_set[3] = theta_3

        return result_set

    def add_world_cs(self, name, world):
        """
        Adds a new world coordinate system
        :param name: the name of the system
        :param world: the new world coordinate system
        """
        self.world_cs[name] = world

    def remove_world_cs(self, name):
        """
        Removes a world coordinate system
        :param name: the name of the system
        """
        self.world_cs.pop(name)

    def get_world_cs(self, name):
        """
        Returns a world coordinate system
        :param name: the name of the system
        :return: the world coordinate system
        """
        return self.world_cs[name]

    @staticmethod
    def cartesian_to_cylindric(x, y, z):
        """
        Converts cartesian coordinates to cylindric coordinates
        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :return: the coordinate in r, z, phi representation
        """

        r = math.sqrt(x ** 2 + y ** 2)
        """
        atan2(y, x):
        https://de.wikipedia.org/wiki/Polarkoordinaten#Berechnung_des_Winkels_im_Intervall_.28.E2.88.92.CF.80.2C_.CF.80.5D
        """
        phi = math.atan2(y, x)

        return r, z, phi

    @staticmethod
    def cylindric_to_cartesian(r, z, phi):
        """
        Converts cylindric coordinates to cartesian coordinates
        :param r: the radius
        :param z: the z-coordinate
        :param phi: the angle of the coordinate (phi)
        :return: the coordinate in x, y, z representation
        """
        x = r * math.cos(phi)
        y = r * math.sin(phi)

        return x, y, z

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


# TODO: to convert tool into cartesian: create a world coordinate system that is aligned "on the tcp" and use that.

class WorldCoordinateSystem:
    # TODO: Needs testing
    """
    Used to create alternate cartesian coordinate systems that can be placed anywhere in
    relation to the base coordinate system
    """

    def __init__(self, x_offset, y_offset, z_offset, theta_x, theta_y, theta_z):
        """
        The coordinate system is placed using offsets of the certain coordinate system axis and
        :param x_offset: the offset of the x-axis
        :param y_offset: the offset of the y-axis
        :param z_offset: the offset of the z-axis
        :param theta_x: the rotation around the x-axis
        :param theta_y: the rotation around the y-axis
        :param theta_z: the rotation around the z-axis
        """

        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.theta_x = theta_x
        self.theta_y = theta_y
        self.theta_z = theta_z

        # Already creating the rotation matrices because the coordinate system will not change
        self.rotate_x = np.array([[1, 0, 0],
                                  [0, math.cos(self.theta_x), -math.sin(self.theta_x)],
                                  [0, math.sin(self.theta_x), math.cos(self.theta_x)]])

        self.rotate_y = np.array([[math.cos(self.theta_y), 0, math.sin(self.theta_y)],
                                  [0, 1, 0],
                                  [-math.sin(self.theta_y), 0, math.cos(self.theta_y)]])

        self.rotate_z = np.array([[math.cos(self.theta_z), -math.sin(self.theta_z), 0],
                                  [math.sin(self.theta_z), math.cos(self.theta_z), 0],
                                  [0, 0, 1]])

        # Rotation matrices for reverse direction (other leading sign)
        self.rotate_x_reverse = np.array([[1, 0, 0],
                                          [0, math.cos(-self.theta_x), -math.sin(-self.theta_x)],
                                          [0, math.sin(-self.theta_x), math.cos(-self.theta_x)]])

        self.rotate_y_reverse = np.array([[math.cos(-self.theta_y), 0, math.sin(-self.theta_y)],
                                          [0, 1, 0],
                                          [-math.sin(-self.theta_y), 0, math.cos(-self.theta_y)]])

        self.rotate_z_reverse = np.array([[math.cos(-self.theta_z), -math.sin(-self.theta_z), 0],
                                          [math.sin(-self.theta_z), math.cos(-self.theta_z), 0],
                                          [0, 0, 1]])

    def world_to_base(self, x, y, z):
        """
        Converts from world coordinates into base coordinates
        :param x: the x-coordinate (world coordinate)
        :param y: the y-coordinate (world coordinate)
        :param z: the z-coordinate (world coordinate)
        :return: The base x, y, z coordinates.
        """
        old_point = np.array([x, y, z])

        # rotate around all axis (x, y, z)
        rotated_x = np.dot(old_point, self.rotate_x_reverse)
        rotated_xy = np.dot(rotated_x, self.rotate_y_reverse)
        rotated_xyz = np.dot(rotated_xy, self.rotate_z_reverse)

        # move
        moved_point = rotated_xyz + np.array([self.x_offset, self.y_offset, self.z_offset])

        return moved_point[0], moved_point[1], moved_point[2]

    def base_to_world(self, x, y, z):
        """
        Converts from base coordinates into world coordinates
        :param x: the x-coordinate (base coordinate)
        :param y: the y-coordinate (base coordinate)
        :param z: the z-coordinate (base coordinate)
        :return: The world x, y, z coordinates.
        """
        old_point = np.array([x, y, z])

        # move
        moved_point = old_point - np.array([self.x_offset, self.y_offset, self.z_offset])

        # rotate around all axis (x, y, z)
        rotated_x = np.dot(moved_point, self.rotate_x)
        rotated_xy = np.dot(rotated_x, self.rotate_y)
        rotated_xyz = np.dot(rotated_xy, self.rotate_z)

        return rotated_xyz[0], rotated_xyz[1], rotated_xyz[2]


class KinematicsError(Exception):
    """
    Is thrown when an error occurs within the kinematics module
    """

    def __init__(self, message):
        """
        Initializes the exception
        :param message: explanation of the error
        """

        self.message = message


class CalculationError(KinematicsError):
    """
    Is thrown when an error occurs during a calculation.
    This mostly occurs when the given point cannot be reached.
    """
    pass


class ParameterError(KinematicsError):
    """
    Is thrown if there is a problem with the given parameters.
    For example when the Base axis is the fixed axis during an inverse kinematics calculation.
    """
    pass
