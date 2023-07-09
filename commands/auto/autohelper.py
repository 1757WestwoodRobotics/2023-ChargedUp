from typing import List
from pathplannerlib import PathPlanner, PathPlannerTrajectory
from wpilib import DataLogManager

import constants


def trajectoryFromFile(name: str) -> List[PathPlannerTrajectory]:
    DataLogManager.log(f"Loading path {name}")
    return PathPlanner.loadPathGroup(
        name,
        constants.kMaxForwardLinearVelocity,
        constants.kMaxForwardLinearAcceleration/2
    )
