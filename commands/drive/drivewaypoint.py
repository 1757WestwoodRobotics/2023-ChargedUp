from math import pi
from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.trajectory import (
    TrapezoidProfile,
    TrapezoidProfileRadians,
)
from wpimath.geometry import Pose2d
from wpimath.controller import (
    ProfiledPIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.kinematics import ChassisSpeeds
from operatorinterface import AnalogInput
from subsystems.drivesubsystem import DriveSubsystem

import constants


class DriveWaypoint(CommandBase):
    def __init__(
        self, drive: DriveSubsystem, xOffset: AnalogInput, yOffset: AnalogInput
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive

        self.xController = ProfiledPIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
        )
        self.yController = ProfiledPIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
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

        self.waypoint = Pose2d()

        SmartDashboard.putData(
            constants.kTargetWaypointXControllerKey, self.xController
        )
        SmartDashboard.putData(
            constants.kTargetWaypointYControllerKey, self.yController
        )
        SmartDashboard.putData(
            constants.kTargetWaypointThetaControllerKey, self.thetaController
        )

        self.xoff = xOffset
        self.yoff = yOffset

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def initialize(self) -> None:
        currentPose = self.drive.getPose()
        currentVel = Pose2d(
            *SmartDashboard.getNumberArray(constants.kDriveVelocityKeys, [0, 0, 0])
        )
        self.xController.reset(currentPose.X(), currentVel.X())
        self.yController.reset(currentPose.Y(), currentVel.Y())
        self.thetaController.reset(
            currentPose.rotation().radians(), currentVel.rotation().radians()
        )

        self.waypoint = min(
            constants.kPossibleWaypoints,
            key=lambda x: x.relativeTo(currentPose).translation().norm(),
        )
        SmartDashboard.putNumberArray(
            constants.kTargetWaypointPoseKey,
            [self.waypoint.X(), self.waypoint.Y(), self.waypoint.rotation().radians()],
        )

    def execute(self) -> None:
        currentPose = self.drive.getPose()
        adjustedWaypointPose = Pose2d(
            self.waypoint.X() + self.xoff() * constants.kWaypointJoystickVariation,
            self.waypoint.Y() + self.yoff() * constants.kWaypointJoystickVariation,
            self.waypoint.rotation(),
        )
        absoluteOutput = ChassisSpeeds(
            self.xController.calculate(currentPose.X(), adjustedWaypointPose.X()),
            self.yController.calculate(currentPose.Y(), adjustedWaypointPose.Y()),
            self.thetaController.calculate(
                currentPose.rotation().radians(),
                adjustedWaypointPose.rotation().radians(),
            ),
        )
        self.drive.arcadeDriveWithSpeeds(
            absoluteOutput, DriveSubsystem.CoordinateMode.FieldRelative
        )

    def end(self, _interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
