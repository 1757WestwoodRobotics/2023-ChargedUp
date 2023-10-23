from enum import Enum, auto
from commands2 import SubsystemBase

from ctre.led import CANdle, RainbowAnimation, ColorFlowAnimation, StrobeAnimation
from wpilib import RobotState, Timer
from wpilib._wpilib import SmartDashboard

import constants


class LightSubsystem(SubsystemBase):
    class State(Enum):
        Cone = auto()
        ConeFlange = auto()
        Cube = auto()
        Holding = auto()

        No = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.light = CANdle(constants.kCANdleID, constants.kCANivoreName)

        self.disabledAnimation1 = RainbowAnimation(1, 0.5, 17, ledOffset=8)
        self.disabledAnimation2 = RainbowAnimation(1, 0.5, 17, ledOffset=8 + 17)
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

        self.coneFlangeAnimation1 = StrobeAnimation(255, 255, 0, 255, 0.3, 17, 8)
        self.coneFlangeAnimation2 = StrobeAnimation(255, 255, 0, 255, 0.3, 17, 8 + 17)

        self.estopAnim1 = StrobeAnimation(255, 0, 0, 255, 0.3, 17, 8)
        self.estopAnim2 = StrobeAnimation(255, 0, 0, 255, 0.3, 17, 8 + 17)

        self.holdingAmination1 = StrobeAnimation(0, 255, 0, 255, 0.3, 17, 8)
        self.holdingAnimation2 = StrobeAnimation(0, 255, 0, 255, 0.3, 17, 8 + 17)

        self.state = LightSubsystem.State.No

        self.hadGamepiece = False
        self.gamepieceTimer = Timer()

    def periodic(self) -> None:
        holdingGamepiece = SmartDashboard.getBoolean(constants.kIntakeMotorHoldingKey, False)

        if RobotState.isEStopped():
            self.light.animate(self.estopAnim1)
            self.light.animate(self.estopAnim2, 1)
        elif RobotState.isDisabled():
            self.light.animate(self.disabledAnimation1)
            self.light.animate(self.disabledAnimation2, 1)
        else:
            if holdingGamepiece and not self.hadGamepiece:
                self.gamepieceTimer.reset()
                self.gamepieceTimer.start()

            if not self.gamepieceTimer.hasElapsed(2):
                self.light.animate(self.holdingAmination1)
                self.light.animate(self.holdingAnimation2, 1)
                SmartDashboard.putString(constants.kLightStateKey, str(LightSubsystem.State.Holding))
            else:
                SmartDashboard.putString(constants.kLightStateKey, str(self.state))
                if self.state == LightSubsystem.State.No:
                    self.light.animate(self.disabledAnimation1)
                    self.light.animate(self.disabledAnimation2, 1)
                elif self.state == LightSubsystem.State.Cone:
                    self.light.animate(self.coneAnimation1)
                    self.light.animate(self.coneAnimation2, 1)
                elif self.state == LightSubsystem.State.Cube:
                    self.light.animate(self.cubeAnimation1)
                    self.light.animate(self.cubeAnimation2, 1)
                elif self.state == LightSubsystem.State.ConeFlange:
                    self.light.animate(self.coneFlangeAnimation1)
                    self.light.animate(self.coneFlangeAnimation2, 1)

        self.hadGamepiece = holdingGamepiece

    def offLights(self) -> None:
        self.state = LightSubsystem.State.No

    def coneLights(self) -> None:
        self.state = LightSubsystem.State.Cone

    def coneFlangeLights(self) -> None:
        self.state = LightSubsystem.State.ConeFlange

    def cubeLights(self) -> None:
        self.state = LightSubsystem.State.Cube
