from commands2 import CommandBase
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from subsystems.drivesubsystem import DriveSubsystem
from util import convenientmath
import constants


class AutoBalance(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.pid = ProfiledPIDController(
            0.025,
            0,
            0,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
        )
        self.drive = drive
        self.pitch = self.drive.gyro.getPitch()
        self.pid.setTolerance(0.001)

    def execute(self) -> None:
        self.pitch = self.drive.gyro.getPitch()
        pidOutput = convenientmath.clamp(
            self.pid.calculate(self.drive.gyro.getPitch(), 0), -0.4, 0.4
        )
        self.drive.arcadeDriveWithFactors(
            0, pidOutput, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def end(self, _interupted: bool) -> None:
        print("IT WORKS")

    def isFinished(self) -> bool:
        return self.pitch > -15 and self.pitch < 15
