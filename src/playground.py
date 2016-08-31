import time
import zmq

from hedgehog.client import HedgehogClient
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

    # Todo: "Calibrate Object"
    links = [26, 22, 12.5]
    base_offset = 5.5
    min_angles = []
    max_angles = []
    min_servo_pos = []
    max_servo_pos = []

    with HedgehogClient(endpoint, ctx=zmq.Context()) as client:
        robotarm = Robotarm(client, links, base_offset, min_angles, max_angles, min_servo_pos, max_servo_pos)
        print("Robotarm init!")

        client.set_multi_servo([(0, False, 0), (1, False, 0), (2, False, 0), (3, False, 0)])

        robotarm.move_to_pos(1800, 0)

        #time.sleep(0.5)

        print("Start")
        robotarm.move_servo_over_time(0, 0, 20)
        print("End")

        robotarm.move_servo_over_time(2000, 0, 10)

        robotarm.move_servo_over_time(500, 0, 5)

        robotarm.close()

        # cfg = [(0, True, 1000), (1, True, 1000)]
        # client.set_multi_servo(cfg)

        # do some tests

if __name__ == "__main__":
    main()
