from math import floor
from os import path
from typing import List
from commands2 import CommandBase
from wpilib import SmartDashboard, Timer
from wpimath.geometry import Rotation2d

import constants
from subsystems.armsubsystem import ArmSubsystem
from util.angleoptimize import optimizeAngle


class DemoArm(CommandBase):
    def __init__(self, arm: ArmSubsystem) -> None:
        CommandBase.__init__(self)
        with open(
            path.join(
                path.dirname(path.realpath(__file__)), constants.kArmDemoFilename
            ),
            "r",
            encoding="utf-8",
        ) as file:
            demoFile = file.readlines()

        self.armFile = ArmFile(demoFile)

        self.arm = arm

        self.timer = Timer()

        self.setName(__class__.__name__)
        self.addRequirements([self.arm])

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()
        self.arm.state = ArmSubsystem.ArmState.OverrideValue

    def execute(self) -> None:
        state = self.armFile.sample(self.timer.get())

        SmartDashboard.putNumber(constants.kElbowArmOverrideKey, state.elbow.degrees())
        SmartDashboard.putNumber(
            constants.kShoulderArmOverrideKey, state.shoulder.degrees()
        )
        SmartDashboard.putNumber(constants.kWristArmOverrideKey, state.wrist.degrees())


class ArmFile:
    class ArmState:
        def __init__(self, shoulder: Rotation2d, elbow: Rotation2d, wrist: Rotation2d):
            self.shoulder = shoulder
            self.elbow = elbow
            self.wrist = wrist

        def lerp(self, other, t: float):
            return ArmFile.ArmState(
                self.shoulder + (other.shoulder - self.shoulder) * t,
                self.elbow + (other.shoulder - self.shoulder) * t,
                self.wrist + (other.wrist - self.wrist) * t,
            )

    def __init__(self, demoFile: List[str]) -> None:
        self.tickrate = 0.041667  # seconds

        self.totalTime = len(demoFile) * self.tickrate

        self.states = []
        for line in demoFile:
            shoulder, elbow, wrist = line.split(" ")
            self.states.append(
                ArmFile.ArmState(
                    optimizeAngle(
                        Rotation2d.fromDegrees(90),
                        Rotation2d.fromDegrees(float(shoulder) + 90),
                    ),
                    optimizeAngle(Rotation2d(), Rotation2d.fromDegrees(float(elbow))),
                    optimizeAngle(Rotation2d(), Rotation2d.fromDegrees(float(wrist))),
                )
            )

    def sample(self, time: float) -> ArmState:
        time = time % self.totalTime  # loop back around

        lowestStateTime = floor(time / self.tickrate)

        lowerState = self.states[lowestStateTime]

        return lowerState
