# each roller alternates meaning if you were to pick up a cube you would eject a cone
from enum import Enum, auto
from commands2 import SubsystemBase
from rev import SparkMaxLimitSwitch
from wpilib import SmartDashboard

from util.simneo import NEOBrushless

import constants


class GripperSubsystem(SubsystemBase):
    class GripperState(Enum):
        Intake = auto()  # Intake (wheels move right)
        Outtake = auto()  # Outtake (wheels move left)
        HoldingState = (
            auto()
        )  # holds the gamepiece in the gripper, no movement in the motor

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.motorIntake = NEOBrushless(
            constants.kIntakeCANID,
            constants.kIntakePIDSlot,
            constants.kIntakePGain,
            constants.kIntakeIGain,
            constants.kIntakeDGain,
            enableLimitSwitches=False,
            limitSwitchPolarity=SparkMaxLimitSwitch.Type.kNormallyClosed,
        )
        self.motorIntake.setSmartCurrentLimit(limit=constants.kIntakeMotorAMPS)

        self.state = GripperSubsystem.GripperState.HoldingState
        self.cubeSensor = lambda: self.motorIntake.getLimitSwitch(
            NEOBrushless.LimitSwitch.Forwards
        )
        self.coneSensor = lambda: self.motorIntake.getLimitSwitch(
            NEOBrushless.LimitSwitch.Backwards
        )
        SmartDashboard.putBoolean(constants.kCubeModeKey, False)

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kIntakeStateKey, str(self.state))
        SmartDashboard.putNumber(
            constants.kIntakeMotorRPMKey,
            self.motorIntake.get(NEOBrushless.ControlMode.Velocity)
            / constants.kIntakeGearRatio,
        )
        SmartDashboard.putBoolean(constants.kCubeLoadedKey, self.cubeSensor())
        SmartDashboard.putBoolean(constants.kConeLoadedKey, self.coneSensor())
        if self.state == self.GripperState.Intake:
            if not SmartDashboard.getBoolean(constants.kCubeModeKey, False):  # Intake
                self.motorIntake.set(
                    NEOBrushless.ControlMode.Percent,
                    -constants.kIntakeMotorPercent
                    # Motor will move forward (right)
                )
            else:
                self.motorIntake.set(
                    NEOBrushless.ControlMode.Percent, constants.kIntakeMotorPercent
                )
        elif self.state == self.GripperState.Outtake:
            if not SmartDashboard.getBoolean(constants.kCubeModeKey, False):  # Intake
                self.motorIntake.set(
                    NEOBrushless.ControlMode.Percent,
                    -constants.kIntakeMotorPercent
                    # Motor will move backward (left)
                )
            else:
                self.motorIntake.set(
                    NEOBrushless.ControlMode.Percent, constants.kIntakeMotorPercent
                )
        elif self.state == self.GripperState.HoldingState:
            if self.cubeSensor():
                self.motorIntake.set(
                    NEOBrushless.ControlMode.Percent, constants.kIntakeHoldingPercent
                )
            elif self.coneSensor():
                self.motorIntake.set(
                    NEOBrushless.ControlMode.Percent, -constants.kIntakeHoldingPercent
                )
            else:
                self.motorIntake.set(NEOBrushless.ControlMode.Percent, 0)

    def setGripIntake(self) -> None:
        self.state = GripperSubsystem.GripperState.Intake

    def setGripOuttake(self) -> None:
        self.state = GripperSubsystem.GripperState.Outtake

    def setGripHold(self) -> None:
        self.state = GripperSubsystem.GripperState.HoldingState


# Luke said that "each instance of the Falcon class gives reference to that specific motor with that specific canID"
