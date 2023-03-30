from enum import Enum, auto
from commands2 import SubsystemBase

from ctre.led import CANdle, RainbowAnimation, ColorFlowAnimation
from wpilib import RobotState

import constants


class LightSubsystem(SubsystemBase):
    class State(Enum):
        Cone = auto()
        Cube = auto()

        No = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.light = CANdle(constants.kCANdleID, constants.kCANivoreName)

        self.disabledAnimation = RainbowAnimation(1, 0.5, 68)
        self.cubeAnimation1 = ColorFlowAnimation(
            204, 0, 204, 255, 0.7, 17, ledOffset=8
        )  # purple
        self.cubeAnimation2 = ColorFlowAnimation(
            204, 0, 204, 255, 0.7, 17, ledOffset=8 + 17
        )  # purple
        self.coneAnimation1 = ColorFlowAnimation(
            255, 255, 0, 255, 0.7, 17, ledOffset=8
        )  # yellow
        self.coneAnimation2 = ColorFlowAnimation(
            255, 255, 0, 255, 0.7, 17, ledOffset=8 + 17
        )  # yellow

        self.state = LightSubsystem.State.No

    def periodic(self) -> None:
        if RobotState.isDisabled():
            self.light.animate(self.disabledAnimation)
        else:
            if self.state == LightSubsystem.State.No:
                self.light.animate(self.disabledAnimation)
            elif self.state == LightSubsystem.State.Cone:
                self.light.animate(self.coneAnimation1)
                self.light.animate(self.coneAnimation2, 1)
            elif self.state == LightSubsystem.State.Cube:
                self.light.animate(self.cubeAnimation1)
                self.light.animate(self.cubeAnimation2, 1)

    def offLights(self) -> None:
        self.state = LightSubsystem.State.No

    def coneLights(self) -> None:
        self.state = LightSubsystem.State.Cone

    def cubeLights(self) -> None:
        self.state = LightSubsystem.State.Cube
