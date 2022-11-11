from commands2 import SequentialCommandGroup, WaitCommand

import constants

from commands.drivetotarget import DriveToTarget
from commands.drivedistance import DriveDistance
from subsystems.drivesubsystem import DriveSubsystem


class ComplexAuto(SequentialCommandGroup):
    """
    A complex auto command that drives to the target, moves forward, then back
    """

    def __init__(self, drive: DriveSubsystem):
        super().__init__(
            # Drive to the target
            DriveToTarget(drive, constants.kAutoTargetOffset),
            # Drive forward the specified distance
            DriveDistance(
                constants.kAutoFrontwaysDistance,
                constants.kAutoDriveSpeedFactor,
                DriveDistance.Axis.X,
                drive,
            ),
            # Wait a bit
            WaitCommand(constants.kAutoWaitDuration),
            # Drive backward the specified distance
            DriveDistance(
                -1 * constants.kAutoFrontwaysDistance,
                constants.kAutoDriveSpeedFactor,
                DriveDistance.Axis.X,
                drive,
            ),
        )
        self.setName(__class__.__name__)
