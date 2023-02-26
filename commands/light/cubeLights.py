from commands2 import CommandBase

from subsystems.lightsubsystem import LightSubsystem


class CubeLights(CommandBase):
    def __init__(self, lightSubsystem: LightSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.lights = lightSubsystem
        self.addRequirements([self.lights])

    def execute(self) -> None:
        self.lights.cubeLights()

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

    def isFinished(self) -> bool:
        return False

    def end(self, _interrupted: bool) -> None:
        self.lights.offLights()


