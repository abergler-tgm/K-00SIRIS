import time
import zmq
from RobotarmClient import RobotarmClient

from hedgehog.client import HedgehogClient
from hedgehog.protocol.messages import motor
from robotarm import Robotarm


def main():
    """
    This is used to test stuff manually
    """

    """
    Base servo min pos ~20, max ~1980
    1st Axis: tbd.
    2nd Axis: min pos ~100 (probably higher), max ~1810
    3rd Axis: min pos ~20 (probably highe), max ~1920

    """

    endpoint = "tcp://192.168.1.7:10789"

    links = [26, 22, 12.5]
    base_offset = 5.5
    min_angles = []
    max_angles = []
    min_servo_pos = []
    max_servo_pos = []

    # do some tests
    with RobotarmClient(endpoint, ctx=zmq.Context()) as client:
        robotarm = Robotarm(client, links, base_offset, min_angles, max_angles, min_servo_pos, max_servo_pos)
        print("Robotarm init!")

        #cfg = [(0, True, 300), (1, True, 400)]
        cfg = [(0, True, 1000), (1, True, 1000)]
        client.set_multi_servo(cfg)



if __name__ == "__main__":
    main()
