import argparse
from os.path import basename
from time import sleep
import logging

import ntcore

logging.basicConfig(level=logging.DEBUG)

parser = argparse.ArgumentParser(description="Gets the pose of the robot")
parser.add_argument(
    "--local",
    dest="local",
    default=False,
    action="store_true",
    help="use local server instead of robot",
)
parser.add_argument(
    "--host", dest="host", default="10.17.57.2", help="The host IP address"
)

args = parser.parse_args()


inst = ntcore.NetworkTableInstance.getDefault()
identity = basename(__file__)
inst.startClient4(identity)
inst.setServer(args.host if not args.local else "127.0.0.1")

sd = inst.getTable("SmartDashboard")

for i in range(2):
    print(sd.getNumberArray("arm/endeffectorPose", [0, 0, 0, 0, 0])[:3])
    sleep(1)
