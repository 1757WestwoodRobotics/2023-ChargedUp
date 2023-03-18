from commands2 import CommandBase
from subsystems.grippersubsystem import GripperSubsystem


class SetGripperState(CommandBase):
    def __init__(self, gripperSubsystem: GripperSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.grip = gripperSubsystem
        self.addRequirements([self.grip])

    def execute(self) -> None:
        raise NotImplementedError("Must be impletmented by subclass")

    def isFinished(self) -> bool:
        return True


class GripperIntake(SetGripperState):
    def __init__(self, gripperSubsystem: GripperSubsystem) -> None:
        SetGripperState.__init__(self, gripperSubsystem)

    def execute(self) -> None:
        self.grip.setGripIntake()


class GripperOuttake(SetGripperState):
    def __init__(self, gripperSubsystem: GripperSubsystem) -> None:
        SetGripperState.__init__(self, gripperSubsystem)

    def execute(self) -> None:
        self.grip.setGripOuttake()


class GripperHoldingState(SetGripperState):
    def __init__(self, gripperSubsystem: GripperSubsystem) -> None:
        SetGripperState.__init__(self, gripperSubsystem)

    def execute(self) -> None:
        self.grip.setGripHold()
