import typing
from commands2 import CommandBase
from wpilib import DriverStation, Preferences
from subsystems.drivesubsystem import DriveSubsystem


class FieldRelativeDrive(CommandBase):
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

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)
        Preferences.initFloat("Robot Relative Sensitivity", 0.4)

    def execute(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.drive.arcadeDriveWithFactors(
                -self.forward(),
                -self.sideways(),
                self.rotation()
                * Preferences.getFloat("Robot Relative Sensitivity"),  # better control
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
        else:
            self.drive.arcadeDriveWithFactors(
                self.forward(),
                self.sideways(),
                self.rotation()
                * Preferences.getFloat("Robot Relative Sensitivity"),  # better control
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
