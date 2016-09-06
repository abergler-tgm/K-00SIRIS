import math

import zmq

from hedgehog.client import HedgehogClient
from robotarm import Robotarm


def main():
    """
    This is used to test stuff manually
    """

    endpoint = "tcp://169.254.0.2:10789"

    """
    angles:
    Base: 0 - 450° = 0 - 2.5*math.pi
    Axis1: 32 - + 145° = ~+/- 60° = 0.5026548245744 - 2.277654673853
    Axis2: -163° - 0 = ~+/- 90° =  -2.560398012676 - 0
    Axis3: -63 - 46° = ~+/- 45° = -0.9896016858808 - 0.7225663103257

    steps (low angle, high angle):
    Base: ~20 - 1980
    Axis1: 2000 - 0
    Axis2: 1810 - ~100
    Axis3: 1920 - ~20
    Axis4: TODO (-> 0)
    Grabber: TODO (-> 0)
    """
    # TODO: Improve precision
    links = [26, 22, 12.5]
    base_offset = 5.5
    min_angles = [0, 0.5026548245744, -2.560398012676, -0.9896016858808, 0, 0]
    max_angles = [2.5*math.pi, 2.277654673853, 0, 0.7225663103257, 0, 0]
    min_servo_pos = [0, 2000, 1810, 1920, 0, 0]
    max_servo_pos = [1980, 0, 100, 20, 0, 0]

    with HedgehogClient(endpoint, ctx=zmq.Context()) as client:
        robotarm = Robotarm(client, links, base_offset, min_angles, max_angles, min_servo_pos, max_servo_pos)

        # do some tests

if __name__ == "__main__":
    main()
