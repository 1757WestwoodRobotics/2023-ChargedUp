# each roller alternates meaning if you were to pick up a cube you would eject a cone
from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import SmartDashboard

from util.simneo import NEOBrushless

import constants


class GripperSubsystem(SubsystemBase):
    class GripperState(Enum):
        CubeGrabForward = auto()  # Cube gripper wheels, motor will move right
        ConeGrabBackwards = auto()  # Cone gripper wheels, motor will move left
        HoldingState = (
            auto()
        )  # holds the cargo in the gripper, no movement in the motor

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.motorCubeCone = NEOBrushless(
            constants.kConeCubeCANID,
            constants.kConeCubePIDSlot,
            constants.kConeCubePGain,
            constants.kConeCubeIGain,
            constants.kConeCubeDGain,
        )
        self.motorCubeCone.setSmartCurrentLimit(limit=25)

        self.state = GripperSubsystem.GripperState.HoldingState
        self.cubeSensor = lambda: self.motorCubeCone.getLimitSwitch(
            NEOBrushless.LimitSwitch.Forwards
        )
        self.coneSensor = lambda: self.motorCubeCone.getLimitSwitch(
            NEOBrushless.LimitSwitch.Backwards
        )
        SmartDashboard.putBoolean(constants.kCubeMode, False)

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kIntakeState, str(self.state))
        SmartDashboard.putNumber(
            constants.kIntakeMotorRPMKey,
            self.motorCubeCone.get(NEOBrushless.ControlMode.Velocity)
            / constants.kIntakeGearRatio,
        )
        SmartDashboard.putBoolean(constants.kCubeLoaded, self.cubeSensor())
        SmartDashboard.putBoolean(constants.kConeLoaded, self.coneSensor())
        if SmartDashboard.getBoolean(constants.kCubeMode, False):
            self.motorCubeCone.set(
                NEOBrushless.ControlMode.Percent, -constants.kIntakeMotorPercent
            )
        if self.state == self.GripperState.CubeGrabForward:
            if not SmartDashboard.getBoolean(constants.kCubeMode, False):  # Intake
                self.motorCubeCone.set(
                    NEOBrushless.ControlMode.Percent,
                    -constants.kIntakeMotorPercent
                    # Motor will move forward (right)
                )
            elif SmartDashboard.getBoolean(constants.kCubeMode, False):  # Outtake
                self.motorCubeCone.set(
                    NEOBrushless.ControlMode.Percent, constants.kIntakeMotorPercent
                )
        elif self.state == self.GripperState.ConeGrabBackwards:
            if SmartDashboard.getBoolean(constants.kCubeMode, False):  # Intake
                self.motorCubeCone.set(
                    NEOBrushless.ControlMode.Percent,
                    constants.kIntakeMotorPercent
                    # Motor will move backward (left)
                )
            elif not SmartDashboard.getBoolean(constants.kCubeMode, False):
                self.motorCubeCone.set(
                    NEOBrushless.ControlMode.Percent, -constants.kIntakeMotorPercent
                )
        elif self.state == self.GripperState.HoldingState:
            if self.cubeSensor():
                self.motorCubeCone.set(
                    NEOBrushless.ControlMode.Percent, constants.kIntakeHoldingPercent
                )
            elif self.coneSensor():
                self.motorCubeCone.set(
                    NEOBrushless.ControlMode.Percent, -constants.kIntakeHoldingPercent
                )
            else:
                self.motorCubeCone.set(NEOBrushless.ControlMode.Percent, 0)

    def setGripCube(self) -> None:
        self.state = GripperSubsystem.GripperState.CubeGrabForward

    def setGripCone(self) -> None:
        self.state = GripperSubsystem.GripperState.ConeGrabBackwards

    def setGripHold(self) -> None:
        self.state = GripperSubsystem.GripperState.HoldingState


# Luke said that "each instance of the Falcon class gives reference to that specific motor with that specific canID"
