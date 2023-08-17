# each roller alternates meaning if you were to pick up a cube you would eject a cone
from enum import Enum, auto
from ctre import SupplyCurrentLimitConfiguration
from commands2 import SubsystemBase
from util.simfalcon import Falcon
from wpilib import SmartDashboard


import constants


class GripperSubsystem(SubsystemBase):
    class GripperState(Enum):
        Intake = auto()
        Outtake = auto()
        HoldingState = (
            auto()
        )  # holds the gamepiece in the gripper, no movement in the motor

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.motorIntake = Falcon(
            constants.kIntakeCANID,
            constants.kIntakePIDSlot,
            constants.kIntakePGain,
            constants.kIntakeIGain,
            constants.kIntakeDGain,
            True
        )

        self.motorIntake.setNeutralMode(Falcon.NeutralMode.Break)

        self.motorIntake.setCurrentLimit(
            SupplyCurrentLimitConfiguration(
                enable=True,
                currentLimit=20,
                triggerThresholdCurrent=30,
                triggerThresholdTime=0.4,
            )
        )

        self.state = GripperSubsystem.GripperState.HoldingState
        SmartDashboard.putBoolean(constants.kCubeModeKey, False)

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kIntakeStateKey, str(self.state))
        SmartDashboard.putNumber(
            constants.kIntakeMotorRPMKey,
            self.motorIntake.get(Falcon.ControlMode.Velocity)
            / constants.kIntakeGearRatio,
        )
        if self.state == self.GripperState.Intake:
            if not SmartDashboard.getBoolean(constants.kCubeModeKey, False):  # Intake
                self.motorIntake.set(
                    Falcon.ControlMode.Percent,
                    -constants.kIntakeMotorPercent
                    # Motor will move forward (right)
                )
            else:
                self.motorIntake.set(
                    Falcon.ControlMode.Percent, constants.kIntakeMotorPercent
                )
        elif self.state == self.GripperState.Outtake:
            if not SmartDashboard.getBoolean(constants.kCubeModeKey, False):  # Intake
                self.motorIntake.set(
                    Falcon.ControlMode.Percent,
                    constants.kIntakeMotorPercent
                    # Motor will move backward (left)
                )
            else:
                self.motorIntake.set(
                    Falcon.ControlMode.Percent, -constants.kIntakeMotorPercent
                )
        elif self.state == self.GripperState.HoldingState:
            self.motorIntake.set(Falcon.ControlMode.Percent, 0)

    def setGripIntake(self) -> None:
        self.state = GripperSubsystem.GripperState.Intake

    def setGripOuttake(self) -> None:
        self.state = GripperSubsystem.GripperState.Outtake

    def setGripHold(self) -> None:
        self.state = GripperSubsystem.GripperState.HoldingState


# Luke said that "each instance of the Falcon class gives reference to that specific motor with that specific canID"
