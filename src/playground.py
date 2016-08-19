import time
import zmq

from hedgehog.client import HedgehogClient
from hedgehog.protocol.messages import motor
from kinematics import Kinematics


def main():
    """
    This is used to test stuff manually
    """

    links = [22.5, 12.5, 10]
    kinematics = Kinematics(links, 5.5)
    x, y, z = 12, 12, 30
    results = kinematics.inverse(x, y, z, 3, 0)



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
