from commands2 import CommandBase
from wpilib import DataLogManager
from subsystems.armsubsystem import ArmSubsystem


class SetArmPositionMid(CommandBase):
    state = ArmSubsystem.ArmState.Mid

    def __init__(self, armSubsystem: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.arm = armSubsystem
        self.addRequirements([self.arm])

    def initialize(self) -> None:
        DataLogManager.log(f"Arm Start {self.state}")

    def execute(self) -> None:
        self.arm.state = self.state

    def end(self, _interrupted: bool) -> None:
        DataLogManager.log(f"Arm End {self.state}")


class SetArmPositionTop(SetArmPositionMid):
    state = ArmSubsystem.ArmState.Top


class SetArmPositionDoubleSubstation(SetArmPositionMid):
    state = ArmSubsystem.ArmState.HumanStation


class SetArmPositionStored(SetArmPositionMid):
    state = ArmSubsystem.ArmState.Stored


class SetArmPositionOverride(SetArmPositionMid):
    state = ArmSubsystem.ArmState.OverrideValue

class SetArmPositionGroundIntake(SetArmPositionMid):
    state = ArmSubsystem.ArmState.GroundLoading
