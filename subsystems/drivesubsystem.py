from enum import Enum, auto

from typing import Tuple
from commands2 import SubsystemBase
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
    SwerveModulePosition,
)

import constants
from util import convenientmath
from util.angleoptimize import optimizeAngle
from util.ctrecheck import ctreCheckError


class SwerveModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def getSwerveAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelLinearVelocity(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelTotalPosition(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def reset(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def optimizedAngle(self, targetAngle: Rotation2d) -> Rotation2d:
        return optimizeAngle(self.getSwerveAngle(), targetAngle)

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelTotalPosition(), self.getSwerveAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.getWheelLinearVelocity(),
            self.getSwerveAngle(),
        )

    def applyState(self, state: SwerveModuleState) -> None:
        optimizedState = SwerveModuleState.optimize(state, self.getSwerveAngle())

        self.setWheelLinearVelocityTarget(optimizedState.speed)
        if (
            abs(optimizedState.speed) >= constants.kMinWheelLinearVelocity
        ):  # prevent unneccisary movement for what would otherwise not move the robot
            optimizedAngle = self.optimizedAngle(optimizedState.angle)
            self.setSwerveAngleTarget(optimizedAngle)


# pylint: disable-next=abstract-method
class PWMSwerveModule(SwerveModule):
    """
    Implementation of SwerveModule designed for ease of simulation:
        wheelMotor: 1:1 gearing with wheel
        swerveMotor: 1:1 gearing with swerve
        wheelEncoder: wheel distance (meters)
        swerveEncoder: swerve angle (radians)
    """

    def __init__(
        self,
        name: str,
        wheelMotor: PWMVictorSPX,
        swerveMotor: PWMVictorSPX,
        wheelEncoder: Encoder,
        swerveEncoder: Encoder,
    ) -> None:
        SwerveModule.__init__(self, name)
        self.wheelMotor = wheelMotor
        self.swerveMotor = swerveMotor
        self.wheelEncoder = wheelEncoder
        self.swerveEncoder = swerveEncoder

        self.wheelEncoder.setDistancePerPulse(1 / constants.kWheelEncoderPulsesPerMeter)
        self.swerveEncoder.setDistancePerPulse(
            1 / constants.kSwerveEncoderPulsesPerRadian
        )

    def getSwerveAngle(self) -> Rotation2d:
        return Rotation2d(self.swerveEncoder.getDistance())

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        swerveError = swerveAngleTarget.radians() - self.swerveEncoder.getDistance()
        swerveErrorClamped = min(max(swerveError, -1), 1)
        self.swerveMotor.set(swerveErrorClamped)

    def getWheelLinearVelocity(self) -> float:
        return self.wheelEncoder.getRate()

    def getWheelTotalPosition(self) -> float:
        return self.wheelEncoder.getDistance()

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        speedFactor = wheelLinearVelocityTarget / constants.kMaxWheelLinearVelocity
        speedFactorClamped = min(max(speedFactor, -1), 1)
        self.wheelMotor.set(speedFactorClamped)

    def reset(self) -> None:
        pass


class CTRESwerveModule(SwerveModule):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Falcon 500 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    def __init__(
        self,
        name: str,
        driveMotor: WPI_TalonFX,
        driveMotorInverted: bool,
        steerMotor: WPI_TalonFX,
        steerMotorInverted: bool,
        swerveEncoder: CANCoder,
        swerveEncoderOffset: float,
    ) -> None:
        SwerveModule.__init__(self, name)
        self.driveMotor = driveMotor
        self.driveMotorInverted = driveMotorInverted
        self.steerMotor = steerMotor
        self.steerMotorInverted = steerMotorInverted
        self.swerveEncoder = swerveEncoder
        self.swerveEncoderOffset = swerveEncoderOffset

        print(f"Initializing swerve module: {self.name}")
        print(
            f"   Configuring swerve encoder: CAN ID: {self.swerveEncoder.getDeviceNumber()}"
        )

        if not ctreCheckError(
            "configFactoryDefault",
            self.swerveEncoder.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "configSensorInitializationStrategy",
            self.swerveEncoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "configMagnetOffset",
            self.swerveEncoder.configMagnetOffset(
                -1 * self.swerveEncoderOffset,  # invert the offset to zero the encoder
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "configAbsoluteSensorRange",
            self.swerveEncoder.configAbsoluteSensorRange(
                AbsoluteSensorRange.Signed_PlusMinus180,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "setPositionToAbsolute",
            self.swerveEncoder.setPositionToAbsolute(
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print("   ... Done")
        print(f"   Configuring drive motor: CAN ID: {self.driveMotor.getDeviceID()}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.driveMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        self.driveMotor.setInverted(self.driveMotorInverted)
        if not ctreCheckError(
            "config_kP",
            self.driveMotor.config_kP(
                constants.kDrivePIDSlot,
                constants.kDrivePGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.driveMotor.config_kI(
                constants.kDrivePIDSlot,
                constants.kDriveIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.driveMotor.config_kD(
                constants.kDrivePIDSlot,
                constants.kDriveDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_SupplyLim",
            self.driveMotor.configSupplyCurrentLimit(
                constants.kDriveSupplyCurrentLimitConfiguration,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print("   ... Done")

        print(f"   Configuring steer motor: CAN ID: {self.steerMotor.getDeviceID()}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.steerMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        self.steerMotor.setInverted(self.steerMotorInverted)
        if not ctreCheckError(
            "config_kP",
            self.steerMotor.config_kP(
                constants.kSteerPIDSlot,
                constants.kSteerPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.steerMotor.config_kI(
                constants.kSteerPIDSlot,
                constants.kSteerIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.steerMotor.config_kD(
                constants.kSteerPIDSlot,
                constants.kSteerDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print("   ... Done")

        print("... Done")

    def getSwerveAngle(self) -> Rotation2d:
        steerEncoderPulses = self.steerMotor.getSelectedSensorPosition()
        swerveAngle = steerEncoderPulses / constants.kSwerveEncoderPulsesPerRadian
        return Rotation2d(swerveAngle)

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        steerEncoderPulses = (
            swerveAngle.radians()
        ) * constants.kSwerveEncoderPulsesPerRadian
        self.steerMotor.setSelectedSensorPosition(steerEncoderPulses)

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderPulsesTarget = (
            swerveAngleTarget.radians() * constants.kSwerveEncoderPulsesPerRadian
        )
        self.steerMotor.set(ControlMode.Position, steerEncoderPulsesTarget)

    def getWheelLinearVelocity(self) -> float:
        driveEncoderPulsesPerSecond = (
            self.driveMotor.getSelectedSensorVelocity()
            * constants.k100MillisecondsPerSecond
        )
        wheelLinearVelocity = (
            driveEncoderPulsesPerSecond / constants.kWheelEncoderPulsesPerMeter
        )
        return wheelLinearVelocity

    def getWheelTotalPosition(self) -> float:
        driveEncoderPulses = self.driveMotor.getSelectedSensorPosition()
        driveDistance = (
            driveEncoderPulses
            / constants.kSwerveEncoderPulsesPerRadian
            * constants.kWheelRadius
        )
        return driveDistance

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderPulsesPerSecond = (
            wheelLinearVelocityTarget * constants.kWheelEncoderPulsesPerMeter
        )
        self.driveMotor.set(
            ControlMode.Velocity,
            driveEncoderPulsesPerSecond / constants.k100MillisecondsPerSecond,
        )

    def reset(self) -> None:
        swerveEncoderAngle = (
            self.swerveEncoder.getAbsolutePosition() * constants.kRadiansPerDegree
        )
        self.setSwerveAngle(Rotation2d(swerveEncoderAngle))


class DriveSubsystem(SubsystemBase):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()
        TargetRelative = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        SmartDashboard.putBoolean(constants.kRobotPoseArrayKeys.validKey, False)

        if RobotBase.isReal():
            self.frontLeftModule = CTRESwerveModule(
                constants.kFrontLeftModuleName,
                WPI_TalonFX(constants.kFrontLeftDriveMotorId, constants.kCANivoreName),
                constants.kFrontLeftDriveInverted,
                WPI_TalonFX(constants.kFrontLeftSteerMotorId, constants.kCANivoreName),
                constants.kFrontLeftSteerInverted,
                CANCoder(constants.kFrontLeftSteerEncoderId, constants.kCANivoreName),
                constants.kFrontLeftAbsoluteEncoderOffset,
            )
            self.frontRightModule = CTRESwerveModule(
                constants.kFrontRightModuleName,
                WPI_TalonFX(constants.kFrontRightDriveMotorId, constants.kCANivoreName),
                constants.kFrontRightDriveInverted,
                WPI_TalonFX(constants.kFrontRightSteerMotorId, constants.kCANivoreName),
                constants.kFrontRightSteerInverted,
                CANCoder(constants.kFrontRightSteerEncoderId, constants.kCANivoreName),
                constants.kFrontRightAbsoluteEncoderOffset,
            )
            self.backLeftModule = CTRESwerveModule(
                constants.kBackLeftModuleName,
                WPI_TalonFX(constants.kBackLeftDriveMotorId, constants.kCANivoreName),
                constants.kBackLeftDriveInverted,
                WPI_TalonFX(constants.kBackLeftSteerMotorId, constants.kCANivoreName),
                constants.kBackLeftSteerInverted,
                CANCoder(constants.kBackLeftSteerEncoderId, constants.kCANivoreName),
                constants.kBackLeftAbsoluteEncoderOffset,
            )
            self.backRightModule = CTRESwerveModule(
                constants.kBackRightModuleName,
                WPI_TalonFX(constants.kBackRightDriveMotorId, constants.kCANivoreName),
                constants.kBackRightDriveInverted,
                WPI_TalonFX(constants.kBackRightSteerMotorId, constants.kCANivoreName),
                constants.kBackRightSteerInverted,
                CANCoder(constants.kBackRightSteerEncoderId, constants.kCANivoreName),
                constants.kBackRightAbsoluteEncoderOffset,
            )
        else:
            self.frontLeftModule = PWMSwerveModule(
                constants.kFrontLeftModuleName,
                PWMVictorSPX(constants.kSimFrontLeftDriveMotorPort),
                PWMVictorSPX(constants.kSimFrontLeftSteerMotorPort),
                Encoder(*constants.kSimFrontLeftDriveEncoderPorts),
                Encoder(*constants.kSimFrontLeftSteerEncoderPorts),
            )
            self.frontRightModule = PWMSwerveModule(
                constants.kFrontRightModuleName,
                PWMVictorSPX(constants.kSimFrontRightDriveMotorPort),
                PWMVictorSPX(constants.kSimFrontRightSteerMotorPort),
                Encoder(*constants.kSimFrontRightDriveEncoderPorts),
                Encoder(*constants.kSimFrontRightSteerEncoderPorts),
            )
            self.backLeftModule = PWMSwerveModule(
                constants.kBackLeftModuleName,
                PWMVictorSPX(constants.kSimBackLeftDriveMotorPort),
                PWMVictorSPX(constants.kSimBackLeftSteerMotorPort),
                Encoder(*constants.kSimBackLeftDriveEncoderPorts),
                Encoder(*constants.kSimBackLeftSteerEncoderPorts),
            )
            self.backRightModule = PWMSwerveModule(
                constants.kBackRightModuleName,
                PWMVictorSPX(constants.kSimBackRightDriveMotorPort),
                PWMVictorSPX(constants.kSimBackRightSteerMotorPort),
                Encoder(*constants.kSimBackRightDriveEncoderPorts),
                Encoder(*constants.kSimBackRightSteerEncoderPorts),
            )

        self.modules = (
            self.frontLeftModule,
            self.frontRightModule,
            self.backLeftModule,
            self.backRightModule,
        )

        self.kinematics = SwerveDrive4Kinematics(
            constants.kFrontLeftWheelPosition,
            constants.kFrontRightWheelPosition,
            constants.kBackLeftWheelPosition,
            constants.kBackRightWheelPosition,
        )

        # Create the gyro, a sensor which can indicate the heading of the robot relative
        # to a customizable position.
        self.gyro = AHRS.create_spi()

        # Create the an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            Pose2d(),
        )

        self.printTimer = Timer()
        self.vxLimiter = SlewRateLimiter(constants.kDriveAccelLimit)
        self.vyLimiter = SlewRateLimiter(constants.kDriveAccelLimit)

    def resetSwerveModules(self):
        for module in self.modules:
            module.reset()
        self.gyro.reset()
        self.resetOdometryAtPosition(Pose2d())

    def setOdometryPosition(self, pose: Pose2d):
        self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.resetOdometryAtPosition(pose)

    def resetGyro(self, pose: Pose2d):
        self.gyro.reset()
        self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.resetOdometryAtPosition(pose)

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()

    def applyStates(self, moduleStates: Tuple[SwerveModuleState]) -> None:
        (
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState,
        ) = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates, constants.kMaxWheelLinearVelocity
        )

        SmartDashboard.putNumberArray(
            constants.kSwerveExpectedStatesKey,
            [
                frontLeftState.angle.degrees(),
                frontLeftState.speed,
                frontRightState.angle.degrees(),
                frontRightState.speed,
                backLeftState.angle.degrees(),
                backLeftState.speed,
                backRightState.angle.degrees(),
                backRightState.speed,
            ],
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)

    def getRotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.getYaw())

    def resetOdometryAtPosition(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getRotation(),
            pose,
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """

        pastPose = self.odometry.getPose()

        self.odometry.update(
            self.getRotation(),
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )
        robotPose = self.odometry.getPose()

        deltaPose = robotPose - pastPose
        SmartDashboard.putNumberArray(
            constants.kSwerveActualStatesKey,
            [
                self.frontLeftModule.getSwerveAngle().degrees(),
                self.frontLeftModule.getWheelLinearVelocity(),
                self.frontRightModule.getSwerveAngle().degrees(),
                self.frontRightModule.getWheelLinearVelocity(),
                self.backLeftModule.getSwerveAngle().degrees(),
                self.backLeftModule.getWheelLinearVelocity(),
                self.backRightModule.getSwerveAngle().degrees(),
                self.backRightModule.getWheelLinearVelocity(),
            ],
        )
        SmartDashboard.putNumberArray(
            constants.kDriveVelocityKeys,
            [
                deltaPose.X()
                / constants.kRobotUpdatePeriod,  # velocity is delta pose / delta time
                deltaPose.Y() / constants.kRobotUpdatePeriod,
                deltaPose.rotation().radians() / constants.kRobotUpdatePeriod,
            ],
        )

        robotPoseArray = [robotPose.X(), robotPose.Y(), robotPose.rotation().radians()]

        if SmartDashboard.getBoolean(
            constants.kRobotVisionPoseArrayKeys.validKey, False
        ):
            visionPose = Pose2d(
                *SmartDashboard.getNumberArray(
                    constants.kRobotVisionPoseArrayKeys.valueKey, robotPoseArray
                )
            )
            weightedPose = Pose2d(
                visionPose.X() * constants.kRobotVisionPoseWeight
                + robotPose.X() * (1 - constants.kRobotVisionPoseWeight),
                visionPose.Y() * constants.kRobotVisionPoseWeight
                + robotPose.Y() * (1 - constants.kRobotVisionPoseWeight),
                visionPose.rotation() * constants.kRobotVisionPoseWeight
                + robotPose.rotation() * (1 - constants.kRobotVisionPoseWeight),
            )
            self.resetOdometryAtPosition(weightedPose)

        SmartDashboard.putNumberArray(
            constants.kRobotPoseArrayKeys.valueKey, robotPoseArray
        )
        SmartDashboard.putBoolean(constants.kRobotPoseArrayKeys.validKey, True)

        if self.printTimer.hasElapsed(constants.kPrintPeriod):
            print(
                # pylint:disable-next=consider-using-f-string
                "r: {:.1f}, {:.1f}, {:.0f}* fl: {:.0f}* {:.1f} fr: {:.0f}* {:.1f} bl: {:.0f}* {:.1f} br: {:.0f}* {:.1f}".format(
                    robotPose.X(),
                    robotPose.Y(),
                    robotPose.rotation().degrees(),
                    self.frontLeftModule.getSwerveAngle().degrees(),
                    self.frontLeftModule.getWheelLinearVelocity(),
                    self.frontRightModule.getSwerveAngle().degrees(),
                    self.frontRightModule.getWheelLinearVelocity(),
                    self.backLeftModule.getSwerveAngle().degrees(),
                    self.backLeftModule.getWheelLinearVelocity(),
                    self.backRightModule.getSwerveAngle().degrees(),
                    self.backRightModule.getWheelLinearVelocity(),
                )
            )

    def arcadeDriveWithFactors(
        self,
        forwardSpeedFactor: float,
        sidewaysSpeedFactor: float,
        rotationSpeedFactor: float,
        coordinateMode: CoordinateMode,
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param forwardSpeedFactor: the commanded forward movement
        :param sidewaysSpeedFactor: the commanded sideways movement
        :param rotationSpeedFactor: the commanded rotation
        """

        forwardSpeedFactor = convenientmath.clamp(forwardSpeedFactor, -1, 1)
        sidewaysSpeedFactor = convenientmath.clamp(sidewaysSpeedFactor, -1, 1)
        rotationSpeedFactor = convenientmath.clamp(rotationSpeedFactor, -1, 1)

        combinedLinearFactor = Translation2d(
            forwardSpeedFactor, sidewaysSpeedFactor
        ).norm()

        # prevent combined forward & sideways inputs from exceeding the max linear velocity
        if combinedLinearFactor > 1.0:
            forwardSpeedFactor = forwardSpeedFactor / combinedLinearFactor
            sidewaysSpeedFactor = sidewaysSpeedFactor / combinedLinearFactor

        chassisSpeeds = ChassisSpeeds(
            forwardSpeedFactor * constants.kMaxForwardLinearVelocity,
            sidewaysSpeedFactor * constants.kMaxSidewaysLinearVelocity,
            rotationSpeedFactor * constants.kMaxRotationAngularVelocity,
        )

        self.arcadeDriveWithSpeeds(chassisSpeeds, coordinateMode)

    def arcadeDriveWithSpeeds(
        self, chassisSpeeds: ChassisSpeeds, coordinateMode: CoordinateMode
    ) -> None:
        targetAngle = Rotation2d(
            SmartDashboard.getNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, 0
            )
        )

        robotChassisSpeeds = None
        if coordinateMode is DriveSubsystem.CoordinateMode.RobotRelative:
            robotChassisSpeeds = chassisSpeeds
        elif coordinateMode is DriveSubsystem.CoordinateMode.FieldRelative:
            robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vx,
                chassisSpeeds.vy,
                chassisSpeeds.omega,
                self.odometry.getPose().rotation(),
            )
        elif coordinateMode is DriveSubsystem.CoordinateMode.TargetRelative:
            if SmartDashboard.getBoolean(
                constants.kTargetAngleRelativeToRobotKeys.validKey, False
            ):
                robotSpeeds = Translation2d(chassisSpeeds.vx, chassisSpeeds.vy)
                targetAlignedSpeeds = robotSpeeds.rotateBy(targetAngle)
                robotChassisSpeeds = ChassisSpeeds(
                    targetAlignedSpeeds.X(),
                    targetAlignedSpeeds.Y(),
                    chassisSpeeds.omega,
                )
            else:
                robotChassisSpeeds = ChassisSpeeds()

        moduleStates = self.kinematics.toSwerveModuleStates(robotChassisSpeeds)
        self.applyStates(moduleStates)
