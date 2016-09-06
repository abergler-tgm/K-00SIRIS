import zmq

from hedgehog.client import HedgehogClient
from robotarm import RobotarmFactory


def main():
    """
    This is used to test stuff manually
    """

    endpoint = "tcp://169.254.0.2:10789"
    with HedgehogClient(endpoint, ctx=zmq.Context()) as client:
        robotarm = RobotarmFactory.get_00siris_default(client)

        # robotarm.move_to_init()   # !

        # do some tests

if __name__ == "__main__":
    main()
