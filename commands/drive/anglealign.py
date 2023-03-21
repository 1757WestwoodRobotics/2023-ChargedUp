from math import pi
import typing
from commands2 import CommandBase
from wpilib import DriverStation
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

import constants
from util.angleoptimize import optimizeAngle


class AngleAlignDrive(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotationPid = PIDController(
            constants.kRotationPGain, constants.kRotationIGain, constants.kRotationDGain
        )
        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def initialize(self) -> None:
        currentRotation = self.drive.getRotation()
        self.targetRotation = Rotation2d.fromDegrees(round(currentRotation.degrees() / 90) * 90)

    def rotation(self) -> float:
        optimizedDirection = optimizeAngle(
            self.drive.getRotation(), self.targetRotation
        ).radians()
        return self.rotationPid.calculate(
            self.drive.getRotation().radians(), optimizedDirection
        )

    def execute(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
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
