from commands2 import CommandBase
from wpilib import SmartDashboard

from wpimath.kinematics import ChassisSpeeds
from subsystems.drivesubsystem import DriveSubsystem

import constants
import math


class AutoBalance(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.stationAngle = math.inf

        self.addRequirements([self.drive])

    def execute(self) -> None:
        self.stationAngle = (
            self.drive.getRotation().cos() * self.drive.getPitch().radians()
            + self.drive.getRotation().sin() * self.drive.getRoll().radians()
        )

        stationAngularVelocity = (
            self.drive.getRotation().cos() * self.drive.getPitchVelocity()
            + self.drive.getRotation().sin() * self.drive.getRollVelocity()
        )

        SmartDashboard.putNumber("STATIONANGLE", self.stationAngle)
        SmartDashboard.putNumber("STATIONVELOCITY", stationAngularVelocity)

        shouldStop = (
            (self.stationAngle < 0.0)
            and (stationAngularVelocity > constants.kAngularVelocityThreshold)
        ) or (
            (self.stationAngle > 0.0)
            and (stationAngularVelocity < -constants.kAngularVelocityThreshold)
        )

        if shouldStop:
            self.drive.arcadeDriveWithSpeeds(
                ChassisSpeeds(), DriveSubsystem.CoordinateMode.RobotRelative
            )
        else:
            self.drive.arcadeDriveWithSpeeds(
                ChassisSpeeds(
                    constants.kAutoBalanceSpeed
                    * (-1.0 if self.stationAngle > 0.0 else 1.0),
                    0.0,
                    0.0,
                ),
                DriveSubsystem.CoordinateMode.FieldRelative,
            )

    def end(self, _interupted: bool) -> None:
        print("IT WORKS")

    def isFinished(self) -> bool:
        return abs(self.stationAngle) < constants.kAngleThreshold
