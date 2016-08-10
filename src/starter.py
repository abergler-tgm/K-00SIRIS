from kinematics.kinematics import Kinematics

# TODO: Change this to Unit-TestCases


def main():
    """
    Test all kinematic functions
    """
    # test all kinematic functions

    links = [25, 22, 12.5]

    kinematics = Kinematics(links, 5.5)

    test_direct_kinematics(kinematics)
    test_inverse_kinematics(kinematics)


def test_direct_kinematics(kinematics):
    """
    Test direct kinematics here

    :param kinematics: the kinematics module
    """
    # angles = ...
    # kinematics.direct(angles)
    pass


def test_inverse_kinematics(kinematics):
    """
    Test inverse kinematics here

    :param kinematics: the kinematics module
    """

    """
    Measured usable points (x, y, z):

    1) 0° on base, 0° on axis 1, 90° on axis 2, 0° on axis 3, 0° on axis 4:
    26, 0, 36

    2) ...
    ...
    """

    x, y, z = 26, 0, 36
    fixed_joint = 3
    kinematics.inverse(x, y, z, fixed_joint, 0)

if __name__ == "__main__":
    main()
