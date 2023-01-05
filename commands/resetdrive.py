from commands2 import CommandBase
from wpimath.geometry import Pose2d

from wpilib import DataLogManager

from subsystems.drivesubsystem import DriveSubsystem


class ResetDrive(CommandBase):
    def __init__(self, drive: DriveSubsystem, position: Pose2d = Pose2d()) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.position = position
        self.addRequirements([self.drive])

    def initialize(self) -> None:
        DataLogManager.log(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.drive.resetSwerveModules()
        self.drive.setOdometryPosition(self.position)

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
