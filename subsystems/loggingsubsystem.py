from commands2 import SubsystemBase
from wpilib._wpilib import PowerDistribution, SmartDashboard


class LoggingSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.pdh = PowerDistribution(1,PowerDistribution.ModuleType.kRev)

    def periodic(self) -> None:
        SmartDashboard.putData(self.pdh)
