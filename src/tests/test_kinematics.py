import unittest

from kinematics import Kinematics, CalculationError, ParameterError


class KinematicsTest(unittest.TestCase):
    """
    Tests the kinematics module
    """

    def setUp(self):
        print("<Setup START>")
        links = [26, 22, 12.5]
        self.kinematics = Kinematics(links, 0)

    def test_round(self):
        """
        Tests the round function
        """
        print("<Test Round START>")
        to_round = [1.8329342, 1.24351622342, 0.2481955, 4.35892392]

        rounded = self.kinematics.round_results(to_round)

        expected = [1.83, 1.24, 0.25, 4.36]

        self.assertEqual(rounded, expected)

    def test_inverse_kuka_initial(self):
        """
        Tests the following inverse kinematics problem:
        1) "KUKA Initial" Joint 3 fixed:
            0 on base, 1.51 on joint 1, 1.48 on joint 2, 0 on joint 3, 0 on joint 4:
            26, 0, 36
        """
        print("<Test Inverse KUKA initial START>")

        x, y, z = 36, 0, 26
        fixed_joint = 3
        angle = 0
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)

        expected = [0.0, 1.51, 1.51, 0.0]

        result = all((abs(x - y) < 0.01 for x, y in zip(results, expected)))

        self.assertTrue(result)

    def test_inverse_touch_ground(self):
        """
        Tests the following inverse kinematics problem:
        1) "touch ground" Axis 3 fixed:
            0 on base, 0.60 on joint 1, 0.80 on joint 2, 0.79 on joint 3, 0 on joint 4:
            50, 0, 0
        """
        print("<Test touch ground initial START>")

        x, y, z = 50, 0, 0
        fixed_joint = 3
        angle = 0.79
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)
        print(results)
        expected = [0.0, 0.60, 0.80, 0.79]

        result = all((abs(x - y) < 0.01 for x, y in zip(results, expected)))

        self.assertTrue(result)

    def test_inverse_zick_zack(self):
        """
        Tests the following inverse kinematics problem:
        1) "Zick Zack" Axis 3 fixed:
            0 on base, 1.36 on joint 1, 2.46 on joint 2, -2.45 on joint 3, 0 on joint 4:
            18, 0, 18
        """
        print("<Test Zick Zack START>")

        x, y, z = 18, 0, 18
        fixed_joint = 3
        angle = -2.45
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)
        print(results)
        expected = [0.0, 1.36, 2.46, -2.45]

        result = all((abs(x - y) < 0.01 for x, y in zip(results, expected)))

        self.assertTrue(result)

    def test_inverse_rear_low(self):
        """
        Tests the following inverse kinematics problem:
        1) "rear low" Axis 3 fixed:
            0 on base, 1.73 on joint 1, 2.50 on joint 2, -0.75 on joint 3, 0 on joint 4:
            24, 0, 10
        """
        print("<Test rear low START>")

        x, y, z = 24, 0, 10
        fixed_joint = 3
        angle = -0.75
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)
        print(results)
        expected = [0.0, 1.73, 2.5, -0.75]

        result = all((abs(x - y) < 0.01 for x, y in zip(results, expected)))

        self.assertTrue(result)

    def test_inverse_rear_high(self):
        """
        Tests the following inverse kinematics problem:
        1) "rear high" Axis 3 fixed:
            0 on base, 1.78 on joint 1, 1.08 on joint 2, 0.65 on joint 3, 0 on joint 4:
            24, 0, 40
        """
        print("<Test rear high START>")

        x, y, z = 24, 0, 40
        fixed_joint = 3
        angle = 0.65
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)
        print(results)
        expected = [0.0, 1.78, 1.08, 0.65]

        result = all((abs(x - y) < 0.01 for x, y in zip(results, expected)))

        self.assertTrue(result)

    def test_inverse_singularity(self):
        """
        Tests the following inverse kinematics problem:
        1) "singularity" Axis 3 fixed:
            0 on base, 1.85 on joint 1, 1.06 on joint 2, -1.5 on joint 3, 0 on joint 4:
            50, 0, 0
        """
        print("<Test touch singularity START>")

        x, y, z = 0, 0, 50
        fixed_joint = 3
        angle = -1.5
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)
        print(results)
        expected = [0.0, 1.85, 1.06, -1.5]

        result = all((abs(x - y) < 0.01 for x, y in zip(results, expected)))

        self.assertTrue(result)

    def test_inverse_first_axis_fixed(self):
        """
        5) Axis 1 fixed:
            0° on base, 1.35 on axis 1, ~1.1 on axis 2, ~1.03 on axis 3:
            35.9, 0, 22.02
        """
        print("<Test Inverse First Axis Fixed START>")

        x, y, z = 35.9, 0, 22.02
        fixed_joint = 1
        angle = 1.35
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)

        rounded = self.kinematics.round_results(results)

        expected = [0.0, 1.35, 1.1, 1.03]

        self.assertEqual(rounded, expected)

    def test_inverse_too_far(self):
        """
        Tests the following inverse kinematics problem:
        11) "Way too far":
            not reachable
            500, 500, 500
        """
        print("<Test Inverse too far START>")

        x, y, z = 500, 500, 500
        fixed_joint = 3
        self.assertRaises(CalculationError, self.kinematics.inverse, x, y, z, fixed_joint, 0)

    def test_inverse_base_fixed(self):
        """
        Tests the following inverse kinematics problem:
        21) "Base is fixed":
            The base cannot be the fixed one
        """
        print("<Test inverse base fixed START>")
        x, y, z = 26, 0, 36
        fixed_joint = 0

        self.assertRaises(ParameterError, self.kinematics.inverse, x, y, z, fixed_joint, 0)

    def test_direct_1(self):
        """
        Test direct kinematics...

        """
        print("<Test direct START>")

        # TODO
        # angles = ...
        # kinematics.direct(angles)
        self.assertEqual(True, False)

if __name__ == '__main__':
    unittest.main()
