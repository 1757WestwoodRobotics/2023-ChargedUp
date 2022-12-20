# to fully see the implementation details of this mechanism, look at https://www.thebluealliance.com/team/2910/2019


from enum import Enum, auto
from commands2 import SubsystemBase

from util.simfalcon import Falcon

import constants


class BallSubsystem(SubsystemBase):
    class BallState(Enum):
        FloorIntake = auto()  # arm is down, motor is running inwards
        Idle = auto()  # arm is up, motor not running
        HumanStationIntake = auto()  # arm is up, motor running inwards
        Outtake = auto()  # arm is up, motor running outwards

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

        self.intakeMotor = Falcon(
            constants.kBallIntakeCANID,
            constants.kBallIntakePIDSlot,
            constants.kBallIntakePGain,
            constants.kBallIntakeIGain,
            constants.kBallIntakeDGain,
        )
        self.liftMotor = Falcon(
            constants.kBallLiftCANID,
            constants.kBallLiftPIDSlot,
            constants.kBallLiftPGain,
            constants.kBallLiftIGain,
            constants.kBallLiftDGain,
        )

        self.state = BallSubsystem.BallState.Idle

    def periodic(self) -> None:
        if self.state == BallSubsystem.BallState.FloorIntake:
            self.intakeMotor.set(
                Falcon.ControlMode.Velocity,
                constants.kBallRotationAmount * constants.kTalonVelocityPerRPM,
            )
            self.liftMotor.set(
                Falcon.ControlMode.Position, constants.kBallLiftLowerPosition
            )
        elif self.state == BallSubsystem.BallState.Idle:
            self.intakeMotor.set(Falcon.ControlMode.Velocity, 0)
            self.liftMotor.set(
                Falcon.ControlMode.Position, constants.kBallLiftUpperPosition
            )
        elif self.state == BallSubsystem.BallState.HumanStationIntake:
            self.intakeMotor.set(
                Falcon.ControlMode.Velocity,
                constants.kBallRotationAmount * constants.kTalonVelocityPerRPM,
            )
            self.liftMotor.set(
                Falcon.ControlMode.Position, constants.kBallLiftUpperPosition
            )
        elif self.state == BallSubsystem.BallState.Outtake:
            self.intakeMotor.set(
                Falcon.ControlMode.Velocity,
                -constants.kBallRotationAmount * constants.kTalonVelocityPerRPM,
            )
            self.liftMotor.set(
                Falcon.ControlMode.Position, constants.kBallLiftUpperPosition
            )

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setFloorIntake(self) -> None:
        self.state = BallSubsystem.BallState.FloorIntake

    def setIdle(self) -> None:
        self.state = BallSubsystem.BallState.Idle

    def setHumanLoading(self) -> None:
        self.state = BallSubsystem.BallState.HumanStationIntake

    def setOuttake(self) -> None:
        self.state = BallSubsystem.BallState.Outtake
