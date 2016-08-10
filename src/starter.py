from kinematics.kinematics import Kinematics


def main():

    # test all kinematic functions

    links = [25, 22, 12.5]

    kinematics = Kinematics(links, 5.5)

    # angles = ...
    # kinematics.direct(angles)

    # x, y, z = ...
    # kinematics.inverse(x, y, z, fixed_joint, angle)


if __name__ == "__main__":
    main()
