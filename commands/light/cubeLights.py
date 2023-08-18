from commands2 import CommandBase
from wpilib import SmartDashboard

from subsystems.lightsubsystem import LightSubsystem

import constants


class CubeLights(CommandBase):
    def __init__(self, lightSubsystem: LightSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.lights = lightSubsystem
        self.addRequirements([self.lights])

    def execute(self) -> None:
        self.lights.cubeLights()
        SmartDashboard.putBoolean(constants.kCubeModeKey, True)

    def isFinished(self) -> bool:
        return False

    def end(self, _interrupted: bool) -> None:
        self.lights.offLights()


class ConeLights(CommandBase):
    def __init__(self, lightSubsystem: LightSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.lights = lightSubsystem
        self.addRequirements([self.lights])

    def execute(self) -> None:
        self.lights.coneLights()
        SmartDashboard.putBoolean(constants.kCubeModeKey, False)
        SmartDashboard.putBoolean(constants.kFlangeModeKey, False)

    def isFinished(self) -> bool:
        return False

    def end(self, _interrupted: bool) -> None:
        self.lights.offLights()

class ConeFlangeLights(CommandBase):
    def __init__(self, lightSubsystem: LightSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.lights = lightSubsystem
        self.addRequirements([self.lights])

    def execute(self) -> None:
        self.lights.coneFlangeLights()
        SmartDashboard.putBoolean(constants.kCubeModeKey, False)
        SmartDashboard.putBoolean(constants.kFlangeModeKey, True)

    def isFinished(self) -> bool:
        return False

    def end(self, _interrupted: bool) -> None:
        self.lights.offLights()
