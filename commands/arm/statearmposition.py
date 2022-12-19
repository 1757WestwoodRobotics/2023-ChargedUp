from commands2 import CommandBase
from commands.arm.demostate import constants
from subsystems.armsubsystem import ArmSubsystem


class SetArmPosition(CommandBase):
    state = ArmSubsystem.ArmState.Stored

    def __init__(self, armSubsystem: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.arm = armSubsystem
        self.addRequirements([self.arm])

    def execute(self) -> None:
        self.arm.state = self.state

    def isFinished(self) -> bool:
        return self.arm.atTarget()


class SetArmPositionMid(SetArmPosition):
    state = ArmSubsystem.ArmState.Mid


class SetArmPositionTop(SetArmPosition):
    state = ArmSubsystem.ArmState.Top


class SetArmPositionDoubleSubstation(SetArmPosition):
    state = ArmSubsystem.ArmState.DoubleSubstation


class SetArmPositionSingleSubstation(SetArmPosition):
    state = ArmSubsystem.ArmState.SingleSubtation


class SetArmPositionStored(SetArmPosition):
    state = ArmSubsystem.ArmState.Stored


class SetArmPositionOverride(SetArmPosition):
    state = ArmSubsystem.ArmState.OverrideValue


class SetArmPositionGroundIntake(SetArmPosition):
    state = ArmSubsystem.ArmState.GroundLoading


class SetArmPositionGroundCone(SetArmPosition):
    state = ArmSubsystem.ArmState.GroundCone


class SetArmPositionSafeTop(SetArmPosition):
    state = ArmSubsystem.ArmState.TopSafe

    def isFinished(self) -> bool:
        return True


class SetArmPositionSafeGround(SetArmPosition):
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
        self.arm.fudgeFactor += constants.kArmFudgeFactorIncrements

    def isFinished(self) -> bool:
        return self.arm.atTarget()


class DecreaseArmFudge(IncreaseArmFudge):
    def execute(self) -> None:
        self.arm.fudgeFactor -= constants.kArmFudgeFactorIncrements
