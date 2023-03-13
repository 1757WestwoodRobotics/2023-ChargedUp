import functools
import operator
from typing import List

from wpimath.geometry import Pose3d


def convertToSendablePoses(poses: List[Pose3d]) -> List[float]:
    sendablePoses = []
    for point in poses:
        x = point.X()
        y = point.Y()
        z = point.Z()
        rotationQuaternion = point.rotation().getQuaternion()
        w_rot = rotationQuaternion.W()
        x_rot = rotationQuaternion.X()
        y_rot = rotationQuaternion.Y()
        z_rot = rotationQuaternion.Z()

        sendablePoses.append([x, y, z, w_rot, x_rot, y_rot, z_rot])

    return functools.reduce(operator.add, sendablePoses, [])
