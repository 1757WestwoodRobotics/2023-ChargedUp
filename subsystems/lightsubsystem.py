from enum import Enum, auto
from commands2 import SubsystemBase

from ctre.led import CANdle, RainbowAnimation, ColorFlowAnimation

from wpilib import AddressableLED, RobotState

import constants


class LightSubsystem(SubsystemBase):
    class State(Enum):
        Cone = auto()
        Cube = auto()

        No = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.light = AddressableLED(8)

        self.ledBuffer = [AddressableLED.LEDData() for i in range(34)]
        self.light.setLength(len(self.ledBuffer))

        # self.disabledAnimation = RainbowAnimation(1, 0.5, 8)
        # self.cubeAnimation1 = ColorFlowAnimation(204, 0, 204, 255, 0.7, 17, ledOffset=8)  # purple
        # self.cubeAnimation2 = ColorFlowAnimation(204, 0, 204, 255, 0.7, 17, ledOffset=8+17)  # purple
        # self.coneAnimation1 = ColorFlowAnimation(255, 255, 0, 255, 0.7, 17, ledOffset=8)  # yellow
        # self.coneAnimation2 = ColorFlowAnimation(255, 255, 0, 255, 0.7, 17, ledOffset=8+ 17)  # yellow

        self.state = LightSubsystem.State.No

        self.hueRotation = 0

    def rainbow(self) -> None:
        for i in range(34):
            hue = self.hueRotation + (i * 180 / 34) % 180
            self.ledBuffer[i].setHSV(int(hue), 255, 128)
        self.hueRotation += 3
        self.hueRotation %= 180
    def periodic(self) -> None:
        if RobotState.isDisabled():
            self.rainbow()
        else:
            if self.state == LightSubsystem.State.No:
                self.rainbow()
            elif self.state == LightSubsystem.State.Cone:
                for led in self.ledBuffer:
                    led.setRGB(255,255,0)
            elif self.state == LightSubsystem.State.Cube:
                for led in self.ledBuffer:
                    led.setRGB(204,0,204)

        self.light.setData(self.ledBuffer)

    def offLights(self) -> None:
        self.state = LightSubsystem.State.No

    def coneLights(self) -> None:
        self.state = LightSubsystem.State.Cone

    def cubeLights(self) -> None:
        self.state = LightSubsystem.State.Cube
