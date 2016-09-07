import math
import time

from kinematics import Kinematics, KinematicsError, WorldCoordinateSystem


class Robotarm:
    """
    Main controller-class for the 00SIRIS Robotarm
        Hedgehog information:
            -2000 Steps per Servo
    """

    def __init__(self, client, links, base_offset, x_offset, y_offset, z_offset, min_angles, max_angles, min_servo_pos,
                 max_servo_pos):
        """
        Constructor - used for initialization
        move_to_init should be called after initialization to move the robotarm to the initial position.
        Servos can only track position after move_to_init has been called! (=High risk of unexpected behaviour!)

        :param links: the lengths of all links in an array
        :param base_offset: the distance in between the center of the base axis and the first joint
        """

        # Params
        self.client = client
        self.links = links
        self.base_offset = base_offset
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.min_angles = min_angles
        self.max_angles = max_angles
        self.min_servo_pos = min_servo_pos
        self.max_servo_pos = max_servo_pos

        # Set to initial position - move_to_init should be called after initialization!
        # Base, axis 1, axis 2, axis 3, axis 4, grabber
        self.servo_pos = [0, 0, 0, 0, 0, 0]

        self.kinematics = Kinematics(self.links, self.base_offset, self.x_offset, self.y_offset, self.z_offset)

    def move_to_init(self):
        """
        Moves the robotarm to the initial position
        """
        print("Moving to init!")
        self.move_to_config([0.0, 1.51, -1.51, 0.0, 0.0, 0.0])
        print("Successfully moved to init!")

    def move_to_pos(self, pos, joint):
        """
        Moves the given joint to the given position
        :param pos: the position
        :param joint: the joint that will be used
        :raise OutOfReachError: if the given position cannot be reached
        """
        if 0 <= pos <= 2000:  # TODO: maybe this can be changed to servo_min/max but this has to be checked properly
            self.client.set_servo(joint, True, int(pos))
            self.servo_pos[joint] = pos
        else:
            raise OutOfReachError("Position out of reach! (Range 0 to 2000): " + str(pos))

    def move_to_pos_t(self, pos, joint, duration):
        """
        Moves a joint to the given position in exactly the given duration
        :param pos: the position that the servo should move to
        :param joint: the joint that should be moved
        :param duration: the duration that the movement should take
        """
        start_position = self.servo_pos[joint]
        movement = pos - start_position

        print("Starting!")

        if movement == 0:
            print("Already at position")
            return  # Already at position

        print("Starting procedure!")

        time_per_step = abs(duration / movement)
        start_time = time.time()

        while start_time + duration > time.time() and pos != self.servo_pos[joint]:
            crnt_step = round((time.time() - start_time) / time_per_step)
            self.move_to_pos(start_position + crnt_step * math.copysign(1, movement), joint)
            time.sleep(0.001)  # To prevent from overload

    def move_to_multi_pos_t(self, positions, duration):
        """
        Moves the joints to the given position in exactly the given duration.
        If less than 6 positions are given, the first x joints are moved, where x is the number of given positions
        :param positions: the positions that the servos should move to
        :param duration: the duration that the movement should take
        """

        if len(positions) < 1 or len(positions) > 6:
            raise OutOfReachError("1-6 positions required!")

        print("Starting procedure!")

        start_positions = self.servo_pos  # might have to copy array here
        print("List of start positions:")
        print(start_positions)

        movements = [pos - self.servo_pos[joint] for joint, pos in enumerate(positions)]
        print("List of movements:")
        print(movements)

        # If movement = 0 -> Don't move: Set times_per_step to 0 and check for 0 when calculating crnt step
        times_per_step = [abs(duration / movement) if movement != 0 else 0 for movement in movements]

        start_time = time.time()

        while start_time + duration > time.time():
            # If time_per_step = 0 -> Movement = 0 -> Stay at position
            crnt_steps = [
                round((time.time() - start_time) / time_per_step) if time_per_step != 0 else 0 for
                joint, time_per_step in enumerate(times_per_step)]

            cfg = [(joint, True, int(start_positions[joint] + crnt_step * math.copysign(1, movements[joint]))) for
                   joint, crnt_step in enumerate(crnt_steps)]

            # TODO: Just using the first 4 axis for now([:4]). Needs a 2nd hedgehog for more
            self.client.set_multi_servo(cfg[:4])

            time.sleep(0.001)  # To prevent from overload

        # Update positions
        for index in range(len(positions), 6):
            positions.append(self.servo_pos[index])

        self.servo_pos = positions
        print("move multi over time - done")

    def move_to_angle(self, angle, joint):
        """
        Moves a joint to a certain angle
        :param angle: the angle
        :param joint: the joint
        """
        pos = self.angle_to_step(angle, joint)

        self.move_to_pos(pos, joint)

    def move_to_cartesian(self, x, y, z, fixed_joint, fj_angle):
        """
        Moves the robotarm to the given cartesian coordinates.
        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :param fixed_joint: the joint that is fixed
        :param fj_angle: the angle of the fixed joint
        """
        try:
            angles = self.kinematics.inverse(x, y, z, fixed_joint, fj_angle)
            self.move_to_config(angles)
        except KinematicsError as ke:
            print("Position cannot be reached: " + ke.message)

        except OutOfReachError as oore:
            print("The kinematics module was able to calculate a position, but the servos are not able to reach it: " +
                  oore.message)

    def move_to_cartesian_t(self, x, y, z, fixed_joint, fj_angle, duration):
        """
        Moves the robotarm to the given cartesian coordinates.
        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :param fixed_joint: the joint that is fixed
        :param fj_angle: the angle of the fixed joint
        :param duration: the duration of the movement
        """
        try:
            angles = self.kinematics.inverse(x, y, z, fixed_joint, fj_angle)
            self.move_to_config_t(angles, duration)
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
        :param alignment: the alignment of the grabber
        """
        try:
            angles = self.kinematics.inverse_aligned(x, y, z, alignment)
            self.move_to_config(angles)
        except KinematicsError as ke:
            print("Position cannot be reached: " + ke.message)

        except OutOfReachError as oore:
            print("The kinematics module was able to calculate a position, but the servos are not able to reach it: " +
                  oore.message)

    def move_to_cartesian_aligned_t(self, x, y, z, alignment, duration):
        """
        Moves the robotarm to the given cartesian coordinates.
        This one takes the alignment of the grabber towards the x-axis and uses it as the fixed joint.
        :param x: the x-coordinate
        :param y: the y-coordinate
        :param z: the z-coordinate
        :param alignment: the alignment of the grabber
        :param duration: the duration of the movement
        """
        try:
            angles = self.kinematics.inverse_aligned(x, y, z, alignment)
            self.move_to_config_t(angles, duration)
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

        if self.validate_config(angles):
            positions = [self.angle_to_step(angle, joint) for joint, angle in enumerate(angles)]
            cfg = [(joint, True, position) for joint, position in enumerate(positions)]
            print("Moving to:")
            # TODO: Just using the first 4 axis for now([:4]). Needs a 2nd hedgehog for more
            print(cfg[:4])
            self.client.set_multi_servo(cfg[:4])
            print("Movement started, should be done very soon!")

            # Update positions
            for index in range(len(positions), 6):
                positions.append(self.servo_pos[index])

            self.servo_pos = positions
        else:
            raise OutOfReachError("The given configuration cannot be reached!")

    def move_to_config_t(self, angles, duration):
        """
        Move the robotarm to the given configuration in the given time.
        :param angles: the angles of all joints
        :param duration: the duration of the movement
        """

        if self.validate_config(angles):
            positions = [self.angle_to_step(angle, joint) for joint, angle in enumerate(angles)]
            self.move_to_multi_pos_t(positions, duration)  # Already updates the servo positions
        else:
            raise OutOfReachError("The given configuration cannot be reached!")

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
            return 0  # if no min/max angle is set for the servo, position does not matter

        total_steps = self.max_servo_pos[joint] - self.min_servo_pos[joint]
        total_angle = self.max_angles[joint] - self.min_angles[joint]

        """
        >Move angle to 0 ("remove" min)
        >Calculate steps/angle ratio
        >Add min servo pos (offset)
        """
        pos = int((total_steps / total_angle) * (angle - self.min_angles[joint]) + self.min_servo_pos[joint])

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

    def validate_config(self, angles):
        """
        Validates whether the given configuration is reachable.
        :param angles: the angles of all joints
        :return: True if the given configuration is reachable, else: False
        """

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
        if self.min_angles[joint] <= angle <= self.max_angles[joint] or self.min_angles[joint] >= angle >= \
                self.max_angles[joint]:
            print("Angle is valid!")
            return True
        else:
            print("Angle is invalid!")
            return False

    def get_tool_cs(self):
        """
        Returns the tool coordinate system
        :return: the tool coordinate system as WorldCoordinateSystem
        """
        angles = []
        for joint, pos in enumerate(self.servo_pos):
            angles[joint] = self.step_to_angle(pos, joint)

        x, y, z, theta_x, theta_y, theta_z = self.kinematics.direct(angles)
        return WorldCoordinateSystem(x, y, z, theta_x, theta_y, theta_z)

    def servos_off(self):
        """
        Turns off all servos
        """
        self.client.set_multi_servo([(0, False, 0), (1, False, 0), (2, False, 0), (3, False, 0)])


class RobotarmFactory:
    """
    Creates preset robotarm objects
    """

    @staticmethod
    def get_00siris_default(client):
        """
        Returns a default robotarm object that can be used if the default 00SIRIS (2016) is used.
        It is still necessary to move to the initial position using move_to_init() before actually moving the robotarm!

        Angles:
        Base: 0 - 450° = 0 - 2.5*math.pi
        Axis1: 32 - + 145° = ~+/- 60° = 0.5026548245744 - 2.277654673853
        Axis2: -163° - 0 = ~+/- 90° =  -2.560398012676 - 0
        Axis3: -63 - 46° = ~+/- 45° = -0.9896016858808 - 0.7225663103257

        Steps (low angle, high angle):
        Base: ~20 - 1980
        Axis1: 2000 - 0
        Axis2: 1810 - ~100
        Axis3: 1920 - ~20
        Axis4: TODO (-> 0)
        Grabber: TODO (-> 0)
        :return:
        """
        # TODO: Improve precision
        links = [26, 22, 12.5]
        base_offset = 5.5
        height_offset = 10
        min_angles = [0, 0.5026548245744, -2.560398012676, -0.9896016858808, 0, 0]
        max_angles = [2.5 * math.pi, 2.277654673853, 0, 0.7225663103257, 0, 0]
        min_servo_pos = [0, 2000, 1810, 1920, 0, 0]
        max_servo_pos = [1980, 0, 100, 20, 0, 0]

        x_offset = 0
        y_offset = 0
        z_offset = 10

        robotarm = Robotarm(client, links, base_offset, x_offset, y_offset, z_offset, min_angles, max_angles,
                            min_servo_pos, max_servo_pos)
        return robotarm


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
