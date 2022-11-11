import typing

from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

from commands2 import CommandBase
from subsystems.drivesubsystem import DriveSubsystem

import constants


class TargetRelativeDrive(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotation = rotation

        self.angleController = PIDController(
            constants.kTargetRelativeDriveAnglePGain,
            constants.kTargetRelativeDriveAngleIGain,
            constants.kTargetRelativeDriveAngleDGain,
        )

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def execute(self) -> None:
        angleControllerOutput = 0
        if SmartDashboard.getBoolean(
            constants.kTargetAngleRelativeToRobotKeys.validKey, False
        ):
            targetAngle = Rotation2d(
                SmartDashboard.getNumber(
                    constants.kTargetAngleRelativeToRobotKeys.valueKey, 0
                )
            )

            angleControllerOutput = self.angleController.calculate(
                -1 * targetAngle.radians(), 0
            )
        else:
            self.angleController.reset()

        self.drive.arcadeDriveWithFactors(
            self.forward(),
            self.sideways(),
            angleControllerOutput,
            DriveSubsystem.CoordinateMode.TargetRelative,
        )
