from kinematics import Kinematics


def main():
    """
    This is used to test stuff manually
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

    print("Rounded values:")

    rounded = []

    for result in results:
        rounded.append(int((result * 100) + 0.5) / 100.0)

    print("Base: " + str(rounded[0]))
    print("Axis1: " + str(rounded[1]))
    print("Axis2: " + str(rounded[2]))
    print("Axis3: " + str(rounded[3]))


if __name__ == "__main__":
    main()
