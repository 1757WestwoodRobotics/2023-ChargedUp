from commands2 import CommandBase
from wpimath.geometry import Pose2d, Rotation2d

from subsystems.armsubsystem import ArmSubsystem

import constants


class StateArmPosition(CommandBase):
    def __init__(self, armSubsystem: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.arm = armSubsystem

        self.addRequirements([self.arm])

    def execute(self) -> None:
        self.arm.setEndEffectorPosition(
            Pose2d(
                -42.963 * constants.kMetersPerInch,
                41.227 * constants.kMetersPerInch,
                Rotation2d.fromDegrees(200),
            )
        )
