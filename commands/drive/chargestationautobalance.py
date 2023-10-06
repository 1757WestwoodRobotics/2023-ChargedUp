from commands2 import CommandBase
from wpilib import SmartDashboard

from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
from subsystems.drivesubsystem import DriveSubsystem
from util import convenientmath
from util.angleoptimize import optimizeAngle


class AutoBalance(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.pid = PIDController(
            0.35,
            0,
            0.02,
        )
        self.drive = drive
        self.pitch = self.drive.gyro.getPitch()
        # self.pid.setTolerance(0.001)

        self.addRequirements([self.drive])

    def execute(self) -> None:
        self.pitch = optimizeAngle(Rotation2d(), self.drive.getPitch()).radians()
        SmartDashboard.putNumber("GYROPITCH", self.pitch)

        if self.isFinished():
            return

        pidOutput = convenientmath.clamp(self.pid.calculate(self.pitch, 0), -0.4, 0.4)
        self.drive.arcadeDriveWithFactors(
            pidOutput, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def end(self, _interupted: bool) -> None:
        print("IT WORKS")

    def isFinished(self) -> bool:
        return False
