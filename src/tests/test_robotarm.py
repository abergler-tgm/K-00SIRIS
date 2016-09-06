import unittest

import math

from robotarm import RobotarmFactory


class RobotarmTest(unittest.TestCase):
    """
    Tests the robot arm module
    """

    def setUp(self):
        self.robotarm = RobotarmFactory.get_00siris_default(False)

    def test_angle_to_steps_base_min(self):
        print("<Test Angle to Steps Base min START>")
        expected = self.robotarm.min_servo_pos[0]
        calculated = self.robotarm.angle_to_step(self.robotarm.min_angles[0], 0)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_base_max(self):
        print("<Test Angle to Steps Base max START>")
        expected = self.robotarm.max_servo_pos[0]
        calculated = self.robotarm.angle_to_step(self.robotarm.max_angles[0], 0)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_axis_1_min(self):
        print("<Test Angle to Steps Axis 1 min START>")
        expected = self.robotarm.min_servo_pos[1]
        calculated = self.robotarm.angle_to_step(self.robotarm.min_angles[1], 1)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_axis_1_max(self):
        print("<Test Angle to Steps Axis 1 max START>")
        expected = self.robotarm.max_servo_pos[1]
        calculated = self.robotarm.angle_to_step(self.robotarm.max_angles[1], 1)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_axis_2_min(self):
        print("<Test Angle to Steps Axis 2 min START>")
        expected = self.robotarm.min_servo_pos[2]
        calculated = self.robotarm.angle_to_step(self.robotarm.min_angles[2], 2)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_axis_2_max(self):
        print("<Test Angle to Steps Axis 2 max START>")
        expected = self.robotarm.max_servo_pos[2]
        calculated = self.robotarm.angle_to_step(self.robotarm.max_angles[2], 2)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_axis_3_min(self):
        print("<Test Angle to Steps Axis 3 min START>")
        expected = self.robotarm.min_servo_pos[3]
        calculated = self.robotarm.angle_to_step(self.robotarm.min_angles[3], 3)

        self.assertEqual(expected, calculated)

    def test_angle_to_steps_axis_3_max(self):
        print("<Test Angle to Steps Axis 3 max START>")
        expected = self.robotarm.max_servo_pos[3]
        calculated = self.robotarm.angle_to_step(self.robotarm.max_angles[3], 3)

        self.assertEqual(expected, calculated)


if __name__ == '__main__':
    unittest.main()
