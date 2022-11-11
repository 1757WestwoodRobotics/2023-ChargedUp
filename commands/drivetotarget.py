from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from subsystems.drivesubsystem import DriveSubsystem
from util import convenientmath

import constants


class DriveToTarget(CommandBase):
    def __init__(self, drive: DriveSubsystem, targetOffset: Translation2d) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.targetOffset = targetOffset
        self.distanceController = ProfiledPIDController(
            constants.kDriveToTargetDistancePGain,
            constants.kDriveToTargetDistanceIGain,
            constants.kDriveToTargetDistanceDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
        )
        self.distanceController.setTolerance(
            constants.kDriveToTargetDistanceTolerance,
            constants.kDriveToTargetLinearVelocityTolerance,
        )

        self.angleController = ProfiledPIDControllerRadians(
            constants.kDriveToTargetAnglePGain,
            constants.kDriveToTargetAngleIGain,
            constants.kDriveToTargetAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.angleController.setTolerance(
            constants.kDriveToTargetAngleTolerance,
            constants.kDriveToTargetAngularVelocityTolerance,
        )
        self.angleController.enableContinuousInput(
            -1 * constants.kRadiansPerRevolution / 2,
            constants.kRadiansPerRevolution / 2,
        )

        self.goalDistance = 0
        self.goalAngle = Rotation2d()
        self.targetAngle = Rotation2d()

        self.addRequirements([self.drive])

    def updateDistanceAndAngleToGoal(self) -> None:
        if not (
            SmartDashboard.getBoolean(
                constants.kTargetAngleRelativeToRobotKeys.validKey, False
            )
            and SmartDashboard.getBoolean(
                constants.kTargetDistanceRelativeToRobotKeys.validKey, False
            )
            and SmartDashboard.getBoolean(
                constants.kTargetFacingAngleRelativeToRobotKeys.validKey, False
            )
        ):
            return

        targetDistance = SmartDashboard.getNumber(
            constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0
        )

        self.targetAngle = Rotation2d(
            SmartDashboard.getNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, 0
            )
        )

        targetFacingAngle = Rotation2d(
            SmartDashboard.getNumber(
                constants.kTargetFacingAngleRelativeToRobotKeys.valueKey, 0
            )
        )

        targetRelativeToRobot = Pose2d(
            convenientmath.translationFromDistanceAndRotation(
                targetDistance, self.targetAngle
            ),
            targetFacingAngle,
        )
        goalRelativeToRobot = targetRelativeToRobot + Transform2d(
            self.targetOffset, Rotation2d()
        )

        self.goalDistance = goalRelativeToRobot.translation().norm()
        self.goalAngle = convenientmath.rotationFromTranslation(
            goalRelativeToRobot.translation()
        )

    def initialize(self) -> None:
        CommandBase.initialize(self)

        self.updateDistanceAndAngleToGoal()

        self.distanceController.reset(self.goalDistance)
        self.angleController.reset(self.targetAngle.radians())

    def execute(self) -> None:
        self.updateDistanceAndAngleToGoal()

        distanceControllerOutput = self.distanceController.calculate(
            -1 * self.goalDistance, 0
        )

        angleControllerOutput = self.angleController.calculate(
            -1 * self.targetAngle.radians(), 0
        )

        distanceControllerAxisOutputs = (
            convenientmath.translationFromDistanceAndRotation(
                distanceControllerOutput, self.goalAngle
            )
        )

        self.drive.arcadeDriveWithFactors(
            distanceControllerAxisOutputs.X(),
            distanceControllerAxisOutputs.Y(),
            angleControllerOutput,
            DriveSubsystem.CoordinateMode.RobotRelative,
        )

    def end(self, _interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def isFinished(self) -> bool:
        return self.distanceController.atGoal() and self.angleController.atGoal()
