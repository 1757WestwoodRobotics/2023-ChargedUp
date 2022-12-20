from commands2 import CommandBase
from subsystems.ballsubsystem import BallSubsystem


class SetBallState(
    CommandBase
):  # since all ball subsystem commands are the same, subclasses will implement calling while this serves as the base
    def __init__(self, ballSubsystem: BallSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.ball = ballSubsystem
        self.addRequirements([self.ball])

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return True


class BallFloorIntake(SetBallState):
    def __init__(self, ballSubsystem: BallSubsystem) -> None:
        SetBallState.__init__(self, ballSubsystem)

    def execute(self) -> None:
        self.ball.setFloorIntake()


class BallIdle(SetBallState):
    def __init__(self, ballSubsystem: BallSubsystem) -> None:
        SetBallState.__init__(self, ballSubsystem)

    def execute(self) -> None:
        self.ball.setIdle()


class BallHumanStation(SetBallState):
    def __init__(self, ballSubsystem: BallSubsystem) -> None:
        SetBallState.__init__(self, ballSubsystem)

    def execute(self) -> None:
        self.ball.setHumanLoading()


class BallOuttake(SetBallState):
    def __init__(self, ballSubsystem: BallSubsystem) -> None:
        SetBallState.__init__(self, ballSubsystem)

    def execute(self) -> None:
        self.ball.setOuttake()
