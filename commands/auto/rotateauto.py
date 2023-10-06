import math
from commands2 import CommandBase
from wpimath.controller import (
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Transform2d

from subsystems.drivesubsystem import DriveSubsystem

import constants


class RotateAuto(CommandBase):
    def __init__(self, amount, speedFactor, drive: DriveSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.distance = math.copysign(amount, amount * speedFactor)
        self.speedFactor = math.copysign(speedFactor, amount * speedFactor)
        self.drive = drive
        self.addRequirements([self.drive])
        self.targetPose = None
        self.distanceToTarget = None

        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.thetaController.enableContinuousInput(-math.pi, math.pi)

        self.rotationToTarget = math.inf

    def initialize(self) -> None:
        currentPose = self.drive.odometry.getPose()
        self.targetPose = currentPose + Transform2d(0, 0, self.distance)
        self.thetaController.reset(currentPose.rotation().radians(), 0)
        self.updateDistanceToTarget()

    def execute(self) -> None:
        self.updateDistanceToTarget()
        currentPose = self.drive.getRotation()
        self.drive.arcadeDriveWithFactors(
            0,
            0,
            self.thetaController.calculate(
                currentPose.radians(), self.targetPose.rotation().radians()
            ),
            DriveSubsystem.CoordinateMode.RobotRelative,
        )

    # pylint: disable-next=unused-argument
    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def isFinished(self) -> bool:
        return self.rotationToTarget < 0.01

    def updateDistanceToTarget(self) -> None:
        currentPose = self.drive.odometry.getPose()
        self.rotationToTarget = abs(
            currentPose.rotation().radians() - self.targetPose.rotation().radians()
        )
