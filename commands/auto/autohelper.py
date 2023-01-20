from typing import List
from pathplannerlib import PathPlanner, PathPlannerTrajectory

import constants


def trajectoryFromFile(name: str) -> List[PathPlannerTrajectory]:
    return PathPlanner.loadPathGroup(
        name,
        constants.kMaxForwardLinearVelocity,
        constants.kMaxForwardLinearAcceleration,
    )
