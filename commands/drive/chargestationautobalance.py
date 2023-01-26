from enum import Enum, auto

from typing import Tuple
from commands2 import CommandBase
from wpilib import Encoder, PWMVictorSPX, RobotBase, SmartDashboard, Timer
from ctre import (
    AbsoluteSensorRange,
    CANCoder,
    ControlMode,
    SensorInitializationStrategy,
    WPI_TalonFX,
)
from navx import AHRS
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

import constants
from util import convenientmath
from util.angleoptimize import optimizeAngle
from util.ctrecheck import ctreCheckError

from subsystems.drivesubsystem import DriveSubsystem
from commands.drivedistance import DriveDistance


class AutoBalance(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive

    def execute(self) -> None:
        self.pitch = self.drive.gyro.getPitch()

        if self.pitch > (constants.kTiltThresholdAutoBalance):
            DriveDistance(
                constants.kDriveDistanceAutoBalance,
                constants.kSpeedFactorAutoBalance,
                DriveDistance.Axis.X,
                self.drive,
            )
        elif self.pitch < (-constants.kTiltThresholdAutoBalance):
            DriveDistance(
                -constants.kDriveDistanceAutoBalance,
                constants.kSpeedFactorAutoBalance,
                DriveDistance.Axis.X,
                self.drive,
            )

    def end(self, _interupted: bool) -> None:
        print("IT WORKS")

    def isFinished(self) -> bool:
        return self.pitch > -15 and self.pitch < 15
