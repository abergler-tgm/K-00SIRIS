import time
import zmq

from hedgehog.client import HedgehogClient
from hedgehog.protocol.messages import motor
from robotarm import Robotarm


def main():
    """
    This is used to test stuff manually
    """

    service = 'hedgehog_server'
    ctx = zmq.Context.instance()
    endpoint = "tcp://192.168.1.5:10789"

    client = HedgehogClient(endpoint, ctx=zmq.Context())

    links = [26, 22, 12.5]
    base_offset = 5.5
    min_angles = []
    max_angles = []
    min_servo_pos = []
    max_servo_pos = []

    robotarm = Robotarm(client, links, base_offset, min_angles, max_angles, min_servo_pos, max_servo_pos)

    # do some tests

    with HedgehogClient(endpoint, ctx=zmq.Context()) as client:
        print(client.set_motor(0, motor.POWER, 1000))
        time.sleep(1)
        print(client.set_motor(0, motor.BRAKE, 0))
        time.sleep(1)

if __name__ == "__main__":
    main()
