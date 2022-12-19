from commands2 import CommandBase
from subsystems.armsubsystem import ArmSubsystem


class ResetArm(CommandBase):
    def __init__(self, armSubsystem: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.arm = armSubsystem
        self.addRequirements([self.arm])

    def execute(self) -> None:
        self.arm.reset()

    def isFinished(self) -> bool:
        return True
