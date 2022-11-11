from math import pi
from commands2 import Swerve4ControllerCommand
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

from subsystems.drivesubsystem import DriveSubsystem
import constants


class FollowTrajectory(Swerve4ControllerCommand):
    def __init__(self, drive: DriveSubsystem, trajectory: Trajectory) -> None:
        self.drive = drive

        self.xController = PIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
        )
        self.yController = PIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
        )
        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.thetaController.enableContinuousInput(-pi, pi)

        super().__init__(
            trajectory,
            self.drive.getPose,
            self.drive.kinematics,
            self.xController,
            self.yController,
            self.thetaController,
            self.drive.applyStates,
            [self.drive],
        )

    def end(self, _interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
