# each roller alternates meaning if you were to pick up a cube you would eject a cone
from enum import Enum, auto
from commands2 import SubsystemBase
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
        self.motorCubeCone.setSmartCurrentLimit(limit=15)

        self.state = GripperSubsystem.GripperState.HoldingState

    def periodic(self) -> None:
        if self.state == self.GripperState.CubeGrabForward:
            self.motorCubeCone.set(
                NEOBrushless.ControlMode.Velocity,
                constants.kIntakeMotorSpeed
                # Motor will move forward (right)
            )
        elif self.state == self.GripperState.ConeGrabBackwards:
            self.motorCubeCone.set(
                NEOBrushless.ControlMode.Velocity,
                -constants.kIntakeMotorSpeed
                # Motor will move backward (left)
            )
        elif self.state == self.GripperState.HoldingState:
            self.motorCubeCone.set(NEOBrushless.ControlMode.Velocity, 0)

    def setGripCube(self) -> None:
        self.state = GripperSubsystem.GripperState.CubeGrabForward

    def setGripCone(self) -> None:
        self.state = GripperSubsystem.GripperState.ConeGrabBackwards

    def setGripHold(self) -> None:
        self.state = GripperSubsystem.GripperState.HoldingState


# Luke said that "each instance of the Falcon class gives reference to that specific motor with that specific canID"
