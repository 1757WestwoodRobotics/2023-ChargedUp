from os import path

from commands2 import SequentialCommandGroup
from wpimath.trajectory import TrajectoryConfig, TrajectoryUtil
from commands.followtrajectory import FollowTrajectory
from commands.resetgyro import ResetGyro

from subsystems.drivesubsystem import DriveSubsystem
import constants


class TrajectoryAuto(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, autoName: str) -> None:
        self.drive = drive
        trajectoryConfig = TrajectoryConfig(
            constants.kMaxForwardLinearVelocity, constants.kMaxForwardLinearAcceleration
        )
        trajectoryConfig.setKinematics(self.drive.kinematics)

        trajectory = TrajectoryUtil.fromPathweaverJson(
            path.join(
                path.dirname(path.realpath(__file__)),
                "..",
                "deploy",
                "pathplanner",
                "generatedJSON",
                autoName + ".wpilib.json",
            )
        )

        super().__init__(
            ResetGyro(self.drive, trajectory.sample(0).pose),
            FollowTrajectory(self.drive, trajectory),
        )
