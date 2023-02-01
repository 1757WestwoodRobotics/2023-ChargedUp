# each roller alternates meaning if you were to pick up a cube you would eject a cone
from enum import Enum, auto
from commands2 import SubsystemBase

from wpilib import PneumaticsModuleType, Solenoid
from ctre import ControlMode

from util.simfalcon import createMotor
from util.simfalcon import Falcon
from util.simneo import NEOBrushless

import constants


class GripperSubsystem(SubsystemBase):
    class GripperState(Enum):
        cubeGrabForward = auto()  # Cube gripper wheels, motor will move right
        coneGrabBackwards = auto()  # Cone gripper wheels, motor will move left
        holdingState = (
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

    def periodic(self) -> None:
        if self.state == self.GripperState.cubeGrabForward:
            self.motorCubeCone.execute(
                NEOBrushless.ControlMode.Velocity,
                constants.kIntakeMotorSpeed
                # Motor will move forward (right)
            )
        elif self.state == self.GripperState.coneGrabBackwards:
            self.motorCubeCone.execute(
                NEOBrushless.ControlMode.Velocity,
                -constants.kIntakeMotorSpeed
                # Motor will move backward (left)
            )
        elif self.state == self.GripperState.holdingState:
            self.motorCubeCone.execute(NEOBrushless.ControlMode.Velocity, 0)

    def setGripCube(self) -> None:
        self.state = GripperSubsystem.GripperState.cubeGrabForward

    def setGripCone(self) -> None:
        self.state = GripperSubsystem.GripperState.coneGrabBackwards

    def setGripHold(self) -> None:
        self.state = GripperSubsystem.GripperState.holdingState


# Luke said that "each instance of the Falcon class gives reference to that specific motor with that specific canID"
