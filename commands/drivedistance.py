import math
from enum import Enum, auto

from commands2 import CommandBase
from wpimath.geometry import Transform2d

import constants
from subsystems.drivesubsystem import DriveSubsystem


class DriveDistance(CommandBase):
    class Axis(Enum):
        X = auto()
        Y = auto()

    def __init__(
        self, distance, speedFactor, axis: Axis, drive: DriveSubsystem
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.distance = math.copysign(distance, distance * speedFactor)
        self.speedFactor = math.copysign(speedFactor, distance * speedFactor)
        self.axis = axis
        self.drive = drive
        self.addRequirements([self.drive])
        self.targetPose = None
        self.distanceToTarget = None

    def initialize(self) -> None:
        currentPose = self.drive.odometry.getPose()
        if self.axis is DriveDistance.Axis.X:
            self.targetPose = currentPose + Transform2d(self.distance, 0, 0)
        elif self.axis is DriveDistance.Axis.Y:
            self.targetPose = currentPose + Transform2d(0, self.distance, 0)
        self.updateDistanceToTarget()

    def execute(self) -> None:
        self.updateDistanceToTarget()
        if self.axis is DriveDistance.Axis.X:
            self.drive.arcadeDriveWithFactors(
                self.speedFactor, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
            )
        elif self.axis is DriveDistance.Axis.Y:
            self.drive.arcadeDriveWithFactors(
                0, self.speedFactor, 0, DriveSubsystem.CoordinateMode.RobotRelative
            )

    # pylint: disable-next=unused-argument
    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def isFinished(self) -> bool:
        return self.distanceToTarget < constants.kAutoDistanceThreshold

    def updateDistanceToTarget(self) -> None:
        currentPose = self.drive.odometry.getPose()
        self.distanceToTarget = currentPose.translation().distance(
            self.targetPose.translation()
        )
