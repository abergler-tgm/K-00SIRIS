import unittest

from kinematics.kinematics import Kinematics


class MyTestCase(unittest.TestCase):
    """
    Tests the kinematics module
    """

    """
    Measured usable points (x, y, z):

    1) "KUKA Initial":
    0° on base, 0° on axis 1, 90° on axis 2, 0° on axis 3, 0° on axis 4:
    26, 0, 36

    2) "...
    ...

    Impossible points - module should detect errors (x, y, z):

    11) "Way too far":
    not reachable
    500, 500, 500

    Other stuff

    21) "Base is fixed":
    The base cannot be the fixed one

    """

    def setUp(self):
        links = [25, 22, 12.5]
        self.kinematics = Kinematics(links, 5.5)

    def test_direct_1(self):
        """
        Test direct kinematics...

        """

        # TODO
        # angles = ...
        # kinematics.direct(angles)
        self.assertEqual(True, False)

    def test_inverse_kuka_initial(self):
        """
        Tests the following inverse kinematics problem:
        1)  "KUKA Initial":
            0° on base, 0° on axis 1, 90° on axis 2, 0° on axis 3, 0° on axis 4:
            26, 0, 36
        """

        x, y, z = 26, 0, 36
        fixed_joint = 3
        results = self.kinematics.inverse(x, y, z, fixed_joint, 0)

        # TODO

        self.assertEqual(True, False)

    def test_inverse_too_far(self):
        """
        Tests the following inverse kinematics problem:
        11) "Way too far":
            not reachable
            500, 500, 500
        """

        x, y, z = 500, 500, 500
        fixed_joint = 3
        results = self.kinematics.inverse(x, y, z, fixed_joint, 0)

        # TODO

        self.assertEqual(True, False)

    def test_inverse_base_fixed(self):
        """
        Tests the following inverse kinematics problem:
        21) "Base is fixed":
            The base cannot be the fixed one
        """
        x, y, z = 26, 0, 36
        fixed_joint = 0
        results = self.kinematics.inverse(x, y, z, fixed_joint, 0)

        # TODO

        self.assertEqual(True, False)

if __name__ == '__main__':
    unittest.main()
