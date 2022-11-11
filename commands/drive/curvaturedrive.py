"""curvature drive, aka chezy drive
maintains a constant curvature for a given rotation amount"""

import typing
from commands2 import CommandBase
from subsystems.drivesubsystem import DriveSubsystem


class CurvatureDrive(
    CommandBase
):  # Arcade drive is just robot relative, but no sideways
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.rotation = rotation

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def execute(self) -> None:
        self.drive.arcadeDriveWithFactors(
            self.forward(),
            0,
            self.rotation() * abs(self.forward()),
            DriveSubsystem.CoordinateMode.RobotRelative,
        )
