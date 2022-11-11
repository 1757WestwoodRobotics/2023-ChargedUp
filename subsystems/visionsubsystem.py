# this module is a placeholder currently as 2023 vision will include apriltags

from commands2 import SubsystemBase


class VisionSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
