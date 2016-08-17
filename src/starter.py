from kinematics.kinematics import Kinematics


def main():
    """
    Test kinematic functions manually
    """

    links = [25, 22, 12.5]
    kinematics = Kinematics(links, 5.5)

    x, y, z = 26, 0, 36
    fixed_joint = 3
    results = kinematics.inverse(x, y, z, fixed_joint, 0)

    print("Base: " + str(results[0]))
    print("Axis1: " + str(results[1]))
    print("Axis2: " + str(results[2]))
    print("Axis3: " + str(results[3]))


def test_direct_kinematics(kinematics):
    """
    Test direct kinematics here

    :param kinematics: the kinematics module
    """
    # angles = ...
    # kinematics.direct(angles)
    pass


if __name__ == "__main__":
    main()
