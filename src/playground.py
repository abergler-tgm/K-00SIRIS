import time
import zmq

from hedgehog.client import HedgehogClient
from hedgehog.protocol.messages import motor
from kinematics import Kinematics


def main():
    """
    This is used to test stuff manually
    """

    service = 'hedgehog_server'
    ctx = zmq.Context.instance()
    endpoint = "tcp://localhost:44367"

    with HedgehogClient(endpoint, ctx=zmq.Context()) as client:
        print(client.set_motor(0, motor.POWER, 1000))
        time.sleep(1)
        print(client.set_motor(0, motor.BRAKE, 0))
        time.sleep(1)

if __name__ == "__main__":
    main()
