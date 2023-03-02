from math import atan2, pi
import typing
from commands2 import CommandBase
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from subsystems.drivesubsystem import DriveSubsystem
from util.angleoptimize import optimizeAngle
import constants


class AbsoluteRelativeDrive(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
        rotationX: typing.Callable[[], float],
        rotationY: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotationPid = PIDController(
            constants.kRotationPGain, constants.kRotationIGain, constants.kRotationDGain
        )
        self.rotationY = rotationY
        self.rotationX = rotationX

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def rotation(self) -> float:
        targetRotation = atan2(
            self.rotationX(), self.rotationY()
        )  # rotate to be relative to driver
        if self.rotationX() == 0 and self.rotationY() == 0:
            return 0

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            targetRotation += pi

        optimizedDirection = optimizeAngle(
            self.drive.getRotation(), Rotation2d(targetRotation)
        ).radians()
        return self.rotationPid.calculate(
            self.drive.getRotation().radians(), optimizedDirection
        )

    def execute(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # if we're on the other side, switch the controls around
            self.drive.arcadeDriveWithFactors(
                -self.forward(),
                -self.sideways(),
                self.rotation(),
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
        else:
            self.drive.arcadeDriveWithFactors(
                self.forward(),
                self.sideways(),
                self.rotation(),
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
