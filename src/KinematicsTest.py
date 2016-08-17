import unittest

from kinematics.kinematics import Kinematics


class MyTestCase(unittest.TestCase):
    """
    Tests the kinematics module
    """

    """
    Measured usable points (x, y, z):

    1) "KUKA Initial" Axis 3 fixed:
    0° on base, 0° on axis 1, 90° on axis 2, 0° on axis 3:
    26, 0, 36

    2) ...
    ...

    5) Axis 1 fixed:
    0° on base, 1.35 on axis 1, ~1.1 on axis 2, ~1.03 on axis 3:
    35.9, 0, 22.02

    Impossible points - module should detect errors (x, y, z):

    11) "Way too far":
    not reachable
    500, 500, 500

    Other stuff

    21) "Base is fixed":
    The base cannot be the fixed one

    """

    def setUp(self):
        print("<Setup START>")
        links = [25, 22, 12.5]
        self.kinematics = Kinematics(links, 5.5)

    def test_round(self):
        print("<Test Round START>")
        to_round = [1.8329342, 1.24351622342, 0.2481955, 4.35892392]

        rounded = self.kinematics.round_results(to_round)

        expected = [1.83, 1.24, 0.25, 4.36]

        self.assertEqual(rounded, expected)

    def test_inverse_kuka_initial(self):
        """
        Tests the following inverse kinematics problem:
        1) "KUKA Initial" Axis 3 fixed:
            0° on base, 0° on axis 1, 90° on axis 2, 0° on axis 3, 0° on axis 4:
            26, 0, 36
        """
        print("<Test Inverse KUKA initial START>")

        x, y, z = 26, 0, 36
        fixed_joint = 3
        angle = 0
        results = self.kinematics.inverse(x, y, z, fixed_joint, angle)

        rounded = self.kinematics.round_results(results)

        expected = [0.0, 1.51, 1.48, 0.0]

        self.assertEqual(rounded, expected)

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
        results = self.kinematics.inverse(x, y, z, fixed_joint, 0)

        # TODO: Exception expected (check for exception)

        self.assertEqual(True, False)

    def test_inverse_base_fixed(self):
        """
        Tests the following inverse kinematics problem:
        21) "Base is fixed":
            The base cannot be the fixed one
        """
        print("<Test inverse base fixed START>")
        x, y, z = 26, 0, 36
        fixed_joint = 0
        results = self.kinematics.inverse(x, y, z, fixed_joint, 0)

        # TODO: Exception expected (check for exception)

        self.assertEqual(True, False)

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
