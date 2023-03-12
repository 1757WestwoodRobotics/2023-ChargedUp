from commands2 import CommandBase
from commands.arm.demostate import constants
from subsystems.armsubsystem import ArmSubsystem


class SetArmPositionMid(CommandBase):
    state = ArmSubsystem.ArmState.Mid

    def __init__(self, armSubsystem: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.arm = armSubsystem
        self.addRequirements([self.arm])

    def execute(self) -> None:
        self.arm.state = self.state

    def isFinished(self) -> bool:
        return self.arm.atTarget()


class SetArmPositionTop(SetArmPositionMid):
    state = ArmSubsystem.ArmState.Top


class SetArmPositionDoubleSubstation(SetArmPositionMid):
    state = ArmSubsystem.ArmState.DoubleSubstation


class SetArmPositionSingleSubstation(SetArmPositionMid):
    state = ArmSubsystem.ArmState.SingleSubtation


class SetArmPositionStored(SetArmPositionMid):
    state = ArmSubsystem.ArmState.Stored


class SetArmPositionOverride(SetArmPositionMid):
    state = ArmSubsystem.ArmState.OverrideValue


class SetArmPositionGroundIntake(SetArmPositionMid):
    state = ArmSubsystem.ArmState.GroundLoading


class SetArmPositionGroundCone(SetArmPositionMid):
    state = ArmSubsystem.ArmState.GroundCone


class SetArmPositionSafeTop(SetArmPositionMid):
    state = ArmSubsystem.ArmState.TopSafe

    def isFinished(self) -> bool:
        return True

class SetArmPositionSafeGround(SetArmPositionMid):
    state = ArmSubsystem.ArmState.GroundSafe

    def isFinished(self) -> bool:
        return True

class IncreaseArmFudge(CommandBase):
    def __init__(self, armSubsystem: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.arm = armSubsystem
        self.addRequirements([self.arm])

    def execute(self) -> None:
        self.arm.fudgeFactor += constants.kArmFudgeFactorIncremetns

    def isFinished(self) -> bool:
        return self.arm.atTarget()


class DecreaseArmFudge(IncreaseArmFudge):
    def execute(self) -> None:
        self.arm.fudgeFactor -= constants.kArmFudgeFactorIncremetns
