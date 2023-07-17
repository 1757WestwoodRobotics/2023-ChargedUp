from commands2 import CommandBase
from subsystems.intakesubsystem import IntakeSubsystem

class IdleIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.intake = intake
        self.addRequirements([self.intake])

    def execute(self) -> None:
        self.intake.setIdle()

    def isFinished(self) -> bool:
        return True

    def end(self, _interrupted: bool) -> None:
        pass

class IntakeIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.intake = intake
        self.addRequirements([self.intake])

    def execute(self) -> None:
        self.intake.setIntaking()

    def isFinished(self) -> bool:
        return True

    def end(self, _interrupted: bool) -> None:
        pass

class OuttakeIntake(CommandBase):
    def __init__(self, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.intake = intake
        self.addRequirements([self.intake])

    def execute(self) -> None:
        self.intake.setIntaking()

    def isFinished(self) -> bool:
        return True

    def end(self, _interrupted: bool) -> None:
        pass
