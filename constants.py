"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

import math
from ctre import SupplyCurrentLimitConfiguration
from wpimath.geometry import (
    Pose3d,
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
)
from wpimath.system.plant import DCMotor
from commands.arm.demostate import constants

from util.keyorganization import OptionalValueKeys

# Basic units
kInchesPerFoot = 12
"""inches / foot"""

kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""

kMetersPerFoot = kMetersPerInch * kInchesPerFoot
"""meters / foot"""

kRadiansPerRevolution = 2 * math.pi
"""radians / revolution"""

kDegeersPerRevolution = 360
"""degrees / revolution"""

kRadiansPerDegree = kRadiansPerRevolution / kDegeersPerRevolution
"""radians / degree"""

kMillisecondsPerSecond = 1000 / 1
"""milliseconds / second"""

kSecondsPerMinute = 60 / 1
"""seconds / minute"""

kRPMPerAngularVelocity = (1 / kRadiansPerRevolution) * kSecondsPerMinute
"""RPM / (radians / second)"""

# Debug parameters
kPrintFrequency = 2
""" 1 / second"""

kPrintPeriod = 1 / kPrintFrequency
"""seconds"""

# Field Physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.25 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 3.5 * kMetersPerInch
"""meters"""

# Robot Physical parameters
kRobotWidth = 26 * kMetersPerInch
"""meters"""

kRobotLength = 24 * kMetersPerInch
"""meters"""

kSwerveModuleCenterToRobotCenterWidth = 10.375 * kMetersPerInch
"""meters"""
kSwerveModuleCenterToRobotCenterLength = 9.375 * kMetersPerInch
"""meters"""

kSwerveModuleDistanceFromRobotCenter = pow(
    pow(kSwerveModuleCenterToRobotCenterWidth, 2)
    + pow(kSwerveModuleCenterToRobotCenterLength, 2),
    0.5,
)
"""meters (c = (a^2 + b^2) ^ 0.5)"""

kFrontLeftWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kFrontRightWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackLeftWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackRightWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kWheelDiameter = 4 * kMetersPerInch
"""meters"""

kWheelRadius = kWheelDiameter / 2
"""meters"""

kWheelCircumference = kWheelRadius * 2 * math.pi
"""meters"""

kWheelDistancePerRevolution = kWheelCircumference
"""meters / revolution"""

kWheelDistancePerRadian = kWheelDistancePerRevolution / kRadiansPerRevolution
"""meters / radian"""

kDriveGearingRatio = (50 / 14) * (17 / 27) * (45 / 15)
"""dimensionless"""

kSteerGearingRatio = 150 / 7
"""dimensionless"""

kMaxMotorAngularVelocity = DCMotor.falcon500().freeSpeed
"""radians / second"""

kMaxWheelAngularVelocity = kMaxMotorAngularVelocity / kDriveGearingRatio
"""radians / second"""

kMaxWheelLinearVelocity = kWheelDistancePerRadian * kMaxWheelAngularVelocity
"""meters / second"""

kMinWheelLinearVelocity = 0.002
"""meters / second"""

kMaxSteerAngularVelocity = kMaxMotorAngularVelocity / kSteerGearingRatio
"""radians / second"""

kMaxForwardLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxSidewaysLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxRotationAngularVelocity = (
    kMaxWheelLinearVelocity / kSwerveModuleDistanceFromRobotCenter
)
"""radians / second (omega = v / r)"""

kMaxWheelLinearAcceleration = kMaxWheelLinearVelocity / 1
"""meters / second^2"""

kMaxForwardLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxSidewaysLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxRotationAngularAcceleration = kMaxRotationAngularVelocity / 0.5
"""radians / second^2"""

kFrontLeftModuleName = "front_left"
kFrontRightModuleName = "front_right"
kBackLeftModuleName = "back_left"
kBackRightModuleName = "back_right"

kKilogramToLbs = 0.454

# Limelight
kLimelightTargetInvalidValue = 0.0
kLimelightTargetValidValue = 1.0
kLimelightMinHorizontalFoV = Rotation2d.fromDegrees(-29.8)
kLimelightMaxHorizontalFoV = Rotation2d.fromDegrees(29.8)
kLimelightMinVerticalFoV = Rotation2d.fromDegrees(-22.85)
kLimelightMaxVerticalFoV = Rotation2d.fromDegrees(22.85)
kLimelightNetworkTableName = "limelight"
kLimelightTargetValidKey = "tv"
kLimelightTargetHorizontalAngleKey = "tx"
kLimelightTargetVerticalAngleKey = "ty"
kLimelightLEDModeKey = "ledMode"
kLimelightTrackerModuleName = "limelight"
kLimelightRelativeToRobotTransform = Transform3d(
    Pose3d(),
    Pose3d(0.236, 0.206, 0.197, Rotation3d()),
)

kLimelightPoseKey = "vision/limelight"

# CANivore
kCANivoreName = "canivore"

# Motors
kFrontLeftDriveMotorId = 10
kFrontLeftSteerMotorId = 11
kFrontRightDriveMotorId = 12
kFrontRightSteerMotorId = 13
kBackLeftDriveMotorId = 14
kBackLeftSteerMotorId = 15
kBackRightDriveMotorId = 16
kBackRightSteerMotorId = 17

kDriveSupplyCurrentLimitConfiguration = SupplyCurrentLimitConfiguration(
    enable=True, currentLimit=35, triggerThresholdCurrent=60, triggerThresholdTime=0.1
)

# Encoders
kFrontLeftSteerEncoderId = 40
kFrontRightSteerEncoderId = 41
kBackLeftSteerEncoderId = 42
kBackRightSteerEncoderId = 43

kCANcoderPulsesPerRevolution = 4096
"""pulses / revolution"""

kCANcoderPulsesPerRadian = kCANcoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kTalonEncoderPulsesPerRevolution = 2048
"""pulses / revolution"""

kTalonEncoderPulsesPerRadian = kTalonEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kDriveEncoderPulsesPerRadian = kDriveEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerMeter = kDriveEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kWheelEncoderPulsesPerRevolution = kDriveEncoderPulsesPerRevolution * kDriveGearingRatio
"""pulses / revolution"""

kWheelEncoderPulsesPerRadian = kWheelEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kWheelEncoderPulsesPerMeter = kWheelEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kSteerEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kSteerEncoderPulsesPerRadian = kSteerEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kSwerveEncoderPulsesPerRevolution = (
    kSteerEncoderPulsesPerRevolution * kSteerGearingRatio
)
"""pulses / revolution"""

kSwerveEncoderPulsesPerRadian = (
    kSwerveEncoderPulsesPerRevolution / kRadiansPerRevolution
)
"""pulses / radian"""

# CTRE
k100MillisecondsPerSecond = 10 / 1  # there are 10 groups of 100 milliseconds per second
"""100 milliseconds / second
   CTRE reports velocities in units of (quantity / 100 milliseconds)
   This factor is used to convert to (quantity / 1 second)
"""

kTalonVelocityPerRPM = (
    kTalonEncoderPulsesPerRevolution / kSecondsPerMinute
) / k100MillisecondsPerSecond
"""(pulses / 100 milliseconds) / RPM"""

kTalonVelocityPerAngularVelocity = kTalonVelocityPerRPM * kRPMPerAngularVelocity
"""(pulses / 100 milliseconds) / (radians / second)"""

kConfigurationTimeoutLimit = int(5 * kMillisecondsPerSecond)
"""milliseconds"""

kDrivePIDSlot = 0
kDrivePGain = 0.15
kDriveIGain = 0.0
kDriveDGain = 2.0

kSteerPIDSlot = 0
kSteerPGain = 0.6
kSteerIGain = 0.0
kSteerDGain = 12.0

kFrontLeftDriveInverted = False
kFrontRightDriveInverted = False
kBackLeftDriveInverted = False
kBackRightDriveInverted = False

kFrontLeftSteerInverted = True
kFrontRightSteerInverted = True
kBackLeftSteerInverted = True
kBackRightSteerInverted = True

"""
To determine encoder offsets (with robot ON and DISABLED):
  1. Rotate all swerve modules so that the wheels:
     * are running in the forwards-backwards direction
     * have the wheel bevel gears facing inwards towards the
       center-line of the robot
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""
kFrontLeftAbsoluteEncoderOffset = 256.113
"""degrees"""

kFrontRightAbsoluteEncoderOffset = 125.420
"""degrees"""

kBackLeftAbsoluteEncoderOffset = 341.719
"""degrees"""

kBackRightAbsoluteEncoderOffset = 331.260
"""degrees"""

kRobotPoseArrayKeys = OptionalValueKeys("RobotPoseArray")

kRobotVisionPoseWeight = 0.00  # 5% vision data

kDriveVelocityKeys = "robotVelocity"
kDriveAccelLimit = 7
kRobotUpdatePeriod = 1 / 50
"""seconds"""
kLimelightUpdatePeriod = 1 / 10
"""seconds"""

# Vision parameters
kTargetAngleRelativeToRobotKeys = OptionalValueKeys("TargetAngleRelativeToRobot")
kTargetDistanceRelativeToRobotKeys = OptionalValueKeys("TargetDistanceRelativeToRobot")
kTargetFacingAngleRelativeToRobotKeys = OptionalValueKeys(
    "TargetFacingAngleRelativeToRobot"
)
kTargetPoseArrayKeys = OptionalValueKeys("TargetPoseArray")
kRobotVisionPoseArrayKeys = OptionalValueKeys("VisionRobotPose")

kTargetName = "Target"

kApriltagPositionDict = {  # thanks 6328 for FieldConstants!
    1: Pose3d(
        (kMetersPerInch * 610.77),
        (kMetersPerInch * 42.19),
        (kMetersPerInch * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    2: Pose3d(
        (kMetersPerInch * 610.77),
        (kMetersPerInch * 108.19),
        (kMetersPerInch * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    3: Pose3d(
        (kMetersPerInch * 610.77),
        (kMetersPerInch * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (kMetersPerInch * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    4: Pose3d(
        (kMetersPerInch * 636.96),
        (kMetersPerInch * 265.74),
        (kMetersPerInch * 27.38),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    5: Pose3d(
        (kMetersPerInch * 14.25),
        (kMetersPerInch * 265.74),
        (kMetersPerInch * 27.38),
        Rotation3d(),
    ),
    6: Pose3d(
        (kMetersPerInch * 40.45),
        (kMetersPerInch * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (kMetersPerInch * 18.22),
        Rotation3d(),
    ),
    7: Pose3d(
        (kMetersPerInch * 40.45),
        (kMetersPerInch * 108.19),
        (kMetersPerInch * 18.22),
        Rotation3d(),
    ),
    8: Pose3d(
        (kMetersPerInch * 40.45),
        (kMetersPerInch * 42.19),
        (kMetersPerInch * 18.22),
        Rotation3d(),
    ),
}

# Autonomous
kAutoDriveDistance = -8 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5
"""dimensionless"""

kAutoWaitDuration = 1
"""seconds"""

kAutoTargetOffset = Translation2d(2, 0)
"""[meters, meters]"""


# Target relative drive
kTargetRelativeDriveAnglePGain = 1
kTargetRelativeDriveAngleIGain = 0
kTargetRelativeDriveAngleDGain = 0

kRotationPGain = 0.8
kRotationIGain = 0
kRotationDGain = 0

# Drive to Target
kDriveToTargetDistancePGain = 0.5
kDriveToTargetDistanceIGain = 0
kDriveToTargetDistanceDGain = 0

kDriveToTargetAnglePGain = 0.5
kDriveToTargetAngleIGain = 0
kDriveToTargetAngleDGain = 0

kDriveToTargetDistanceTolerance = 10 / kCentimetersPerMeter
"""meters"""

kDriveToTargetLinearVelocityTolerance = 1 / kCentimetersPerMeter / 1
"""meters / second"""

kDriveToTargetAngleTolerance = 5 * kRadiansPerDegree
"""radians"""

kDriveToTargetAngularVelocityTolerance = 5 * kRadiansPerDegree / 1
"""radians / second"""

# Trajectory Following
kTrajectoryPositionPGain = 8
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 8
kTrajectoryAngleIGain = 0
kTrajectoryAngleDGain = 0

# Operator Interface
kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""

kControllerMappingFilename = "ControlScheme.json"

kChassisRotationXAxisName = "chassisXRotation"
kChassisRotationYAxisName = "chassisYRotation"
kChassisForwardsBackwardsAxisName = "chassisForwardsBackwards"
kChassisSideToSideAxisName = "chassisSideToSide"

kFieldRelativeCoordinateModeControlButtonName = "fieldRelativeCoordinateModeControl"
kResetGyroButtonName = "resetGyro"
kTargetRelativeCoordinateModeControlButtonName = "targetRelativeCoordinateModeControl"
kDriveToTargetControlButtonName = "driveToTargetControl"
kXboxTriggerActivationThreshold = 0.5

kTurboSpeedButtonName = "turboSpeed"
kNormalSpeedMultiplier = 0.80  # half full on normal
kTurboSpeedMultiplier = 0.95  # full speed!!!

# Simulation Parameters
kSimTargetName = "SimTarget"
kSimDefaultTargetLocation = Pose2d(
    kFieldLength / 2, kFieldWidth / 2, 180 * kRadiansPerDegree
)
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(kFieldLength / 2, kFieldWidth / 2, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in
kSimBallName = "SimBall"
kSimDefaultBallLocation = Pose2d(kFieldLength / 4, kFieldWidth / 2, 0)

"""meters"""

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimTargetPoseArrayKey = "SimTargetPoseArray"
kSimBallPoseArrayKey = "SimBallPoseArray"
kSimTargetHeightKey = "SimTargetHeight"
kSimTargetTrackingModuleName = "sim_target_tracker"
kSimTargetUpperHubRadius = 2

kSimFrontLeftDriveMotorPort = 0
kSimFrontLeftSteerMotorPort = 1
kSimFrontRightDriveMotorPort = 2
kSimFrontRightSteerMotorPort = 3
kSimBackLeftDriveMotorPort = 4
kSimBackLeftSteerMotorPort = 5
kSimBackRightDriveMotorPort = 6
kSimBackRightSteerMotorPort = 7


kSimFrontLeftDriveEncoderPorts = (16, 1)
kSimFrontLeftSteerEncoderPorts = (2, 3)
kSimFrontRightDriveEncoderPorts = (4, 5)
kSimFrontRightSteerEncoderPorts = (6, 7)
kSimBackLeftDriveEncoderPorts = (8, 9)
kSimBackLeftSteerEncoderPorts = (10, 11)
kSimBackRightDriveEncoderPorts = (12, 13)
kSimBackRightSteerEncoderPorts = (14, 15)


kMotorBaseKey = "motors"

# waypoint setter constraints
kMaxWaypointTranslationalVelocity = kMaxForwardLinearVelocity
kMaxWaypointTranslationalAcceleration = kMaxWaypointTranslationalVelocity * 3

kPossibleWaypoints = [
    Pose2d(15.64, 7.34, 0),  # blue loading station
    Pose2d(15.64, 6.16, 0),
    Pose2d(0.87, 7.34, math.pi),  # red loading staton
    Pose2d(0.87, 6.16, math.pi),
    Pose2d(1.92, 5.00, -math.pi),  # scoring locations, blue
    Pose2d(1.92, 4.42, -math.pi),
    Pose2d(1.92, 3.83, -math.pi),
    Pose2d(1.92, 3.30, -math.pi),
    Pose2d(1.92, 2.76, -math.pi),
    Pose2d(1.92, 2.19, -math.pi),
    Pose2d(1.92, 1.69, -math.pi),
    Pose2d(1.92, 1.09, -math.pi),
    Pose2d(1.92, 0.51, -math.pi),
    Pose2d(14.70, 5.00, 0),  # scoring locations, red
    Pose2d(14.70, 4.42, 0),
    Pose2d(14.70, 3.83, 0),
    Pose2d(14.70, 3.30, 0),
    Pose2d(14.70, 2.76, 0),
    Pose2d(14.70, 2.19, 0),
    Pose2d(14.70, 1.69, 0),
    Pose2d(14.70, 1.09, 0),
    Pose2d(14.70, 0.51, 0),
]
kWaypointJoystickVariation = 0.1
"""meters"""

kTargetWaypointPoseKey = "waypoint/target"
kTargetWaypointXControllerKey = "waypoint/x"
kTargetWaypointYControllerKey = "waypoint/y"
kTargetWaypointThetaControllerKey = "waypoint/theta"
# Arm
kArmPIDSlot = 0

kArmEncoderToSprocketGearRatio = 60 / 12

kElbowArmCANId = 30
kElbowArmPGain = 0.02
kElbowArmIGain = 0.0
kElbowArmDGain = 0
kElbowArmInverted = False

kElbowArmGearRatio = (3 / 1) * (58 / 10) * (58 / 18) * (60 / 12)
kElbowArmRotationKey = "arm/rotation/elbow"
kElbowArmTargetRotationKey = "arm/target/elbow"
kElbowFeedForwardLogKey = "arm/ff/elbow"
kElbowArmFFFactor = 0.0

kElbowArmCANCoderID = 35
kElbowArmCANCoderOffset = 328.11
"""
to get encoder offsets for the arm motors
  1. bring the arm to the position you want to call zero
    * the zero for the arm is with the elbow hitting its endstop 
    * with the shoulder touching the chain barely
    * with the intake buffer touching the CF tube
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""

kElbowMinAngle = Rotation2d.fromDegrees(0)
kElbowMaxAngle = Rotation2d.fromDegrees(145)

kShoulderArmCANId = 31
kShoulderArmPGain = 0.03
kShoulderArmIGain = 0.0
kShoulderArmDGain = 0
kShoulderArmInverted = True

kShoulderArmGearRatio = (4 / 1) * (58 / 10) * (58 / 18) * (60 / 12)
kShoulderArmRotationKey = "arm/rotation/shoulder"
kShoulderTargetArmRotationKey = "arm/target/shoulder"
kShoulderFeedForwardLogKey = "arm/ff/shoulder"
kShoulderArmFFFactor = 0.0

kShoulderArmCANCoderID = 45
kShoulderArmCANCoderOffset = 314.912

kShoulderMinAngle = Rotation2d.fromDegrees(57)
kShoulderMaxAngle = Rotation2d.fromDegrees(157)

kWristArmCANId = 32
kWristArmPGain = 0.03
kWristArmIGain = 0.0
kWristArmDGain = 0
kWristArmInverted = True

kWristArmGearRatio = (3 / 1) * (58 / 10) * (58 / 18) * (60 / 12)
kWristArmRotationKey = "arm/rotation/wrist"
kWristTargetArmRotationKey = "arm/target/wrist"
kWristFeedForwardLogKey = "arm/ff/wrist"
kWristArmFFFactor = 0.0

kWristArmCANCoderID = 46
kWristArmCANCoderOffset = 270.791

kWristMinAngle = Rotation2d.fromDegrees(41 - 180)
kWristMaxAngle = Rotation2d.fromDegrees(180)

kArmElbowLength = 19 * kMetersPerInch
kArmElbowMass = 24.5 * kKilogramToLbs
kArmElbowCOM = 6 * kMetersPerInch

kArmShoulderLength = 27 * kMetersPerInch
kArmShoulderMass = 6.3 * kKilogramToLbs
kArmShoulderCOM = 13 * kMetersPerInch

kArmWristLength = 16.392506 * kMetersPerInch
kArmWristMass = 12.8 * kKilogramToLbs
kArmWristCOM = 7.35 * kMetersPerInch

kArmStateKey = "arm/state"

kArmTranslationalPGain = 1.1
kArmTranslationalIGain = 0
kArmTranslationalDGain = 0

kArmTranslationalMaxVelocity = 10
"""m/s"""
kArmTranslationalMaxAcceleration = 15
"""m/s^2"""

kArmRotationalPGain = 0.9
kArmRotationalIGain = 0
kArmRotationalDGain = 0

kArmRotationalMaxVelocity = 40
"""rad/s"""
kArmRotationalMaxAcceleration = 80
"""rad/s^2"""
kElbowArmOverrideKey = "arm/override/elbow"
kShoulderArmOverrideKey = "arm/override/shoulder"
kWristArmOverrideKey = "arm/override/wrist"

kShoulderRobotOffset = Transform3d(
    Translation3d(8 * kMetersPerInch, 0, 9.75 * kMetersPerInch),
    Rotation3d(0, math.pi, 0),
)
kArmPosesKey = "arm/poses/actual"
kArmTargetPosesKey = "arm/poses/target"
kArmCOMs = "arm/coms"
kArmObeyEndstopsKey = "arm/useEndstops"
kArmInterpolationMethod = "arm/interpolationMethod"

kArmPositionExtraEpsiolon = 0.001

kArmPositionTolerence = 0.1
"""meters"""
kArmPositionStoredTolerence = 0.5
"""meters"""
kArmRotationTolerence = 0.15
"""radians"""
kArmAtTargetKey = "arm/atTarget"

kArmDemoFilename = "demoFile.txt"

kArmShoulderTargetMotorKey = "arm/motors/target/shoulder"
kArmElbowTargetMotorKey = "arm/motors/target/elbow"
kArmWristTargetMotorKey = "arm/motors/target/wrist"

kArmShoulderActualMotorKey = "arm/motors/actual/shoulder"
kArmElbowActualMotorKey = "arm/motors/actual/elbow"
kArmWristActualMotorKey = "arm/motors/actual/wrist"

# scoring positions, derived from cad geometry
kArmTopScorePosition = Pose2d(
    -41.430 * kMetersPerInch,
    46.44 * kMetersPerInch,
    Rotation2d.fromDegrees(138),
)
kArmMidScorePosition = Pose2d(
    -26.345 * kMetersPerInch,
    35.015 * kMetersPerInch,
    Rotation2d.fromDegrees(138),
)
kArmStoredPosition = Pose2d(
    4.5311 * kMetersPerInch,
    30.1323 * kMetersPerInch,
    Rotation2d.fromDegrees(63),
)
kArmStartupPosition = Pose2d(
    8.452357 * kMetersPerInch,
    26.865289 * kMetersPerInch,
    Rotation2d.fromDegrees(51.338),
)
kArmStartupAngles = (
    0.9643281370153848,
    2.618827161508282,
    -2.687138167134818,
)  # radians
kArmDoubleSubstationPositionCube = Pose2d(
    -21.907 * kMetersPerInch,
    42.693 * kMetersPerInch,
    Rotation2d.fromDegrees(143.433),
)
kArmDoubleSubstationPositionCone = Pose2d(
    -20.881 * kMetersPerInch,
    43.701 * kMetersPerInch,
    Rotation2d.fromDegrees(144.344),
)
kArmSingleSubstationPosition = Pose2d(
    -9.679 * kMetersPerInch,
    35.526 * kMetersPerInch,
    Rotation2d.fromDegrees(180 - 61.617),
)
kArmGroundIntakePositionCone = Pose2d(
    -32.484 * kMetersPerInch,
    7.08 * kMetersPerInch,
    Rotation2d.fromDegrees(145.188),
)
kArmGroundIntakePositionCube = Pose2d(
    -37.568 * kMetersPerInch,
    4.98 * kMetersPerInch,
    Rotation2d.fromDegrees(161.239),
)
kArmGroundIntakePositionCubeYoshi = kArmGroundIntakePositionCube + Transform2d(
    20.0 * kMetersPerInch, 0, 0
)  # larger extension outwards to not need as much rotation
kArmGroundSafePosition = Pose2d(
    -30.639 * kMetersPerInch,
    37.087 * kMetersPerInch,
    Rotation2d.fromDegrees(180),
)
kArmTopSafePosition = Pose2d(
    -10.430 * kMetersPerInch,
    49.622 * kMetersPerInch,
    Rotation2d.fromDegrees(51),
)
kArmGroundConeIntakePosition = Pose2d(
    -30.357 * kMetersPerInch, -3.344 * kMetersPerInch, Rotation2d.fromDegrees(-115.685)
)

kArmFudgeFactorIncrements = 0.5 * constants.kMetersPerInch
"""meters"""
kArmFudgeFactorKey = "arm/fudge"

kArmMotorBreakArmModeKey = "arm/motorMode"
kArmEndEffectorPose = "arm/endeffectorPose"

kArmMaxOscillationAmount = Rotation2d.fromDegrees(10)

# lights
kCANdleID = 2


# Logging
kSwerveActualStatesKey = "swerve/actual"
kSwerveExpectedStatesKey = "swerve/expected"
kConsoleLog = "log"
kPDHCanID = 1
kPDHPublishKey = "powerDistribution"

kJoystickKeyLogPrefix = "DriverStation"
kFieldSimTargetKey = "SimTargets"
kFieldRelativeTargets = "RelTargets"
kAutonomousPathKey = "auto/path"
kAutonomousPathSample = "auto/desired"
kAutonomousPathError = "auto/error"
kAutonomousChassisSpeeds = "auto/speeds"

# photonvision parameters
kPhotonvisionCameraName = "cam"
kPhotonvisionAmbiguityCutoff = 0.2
"""dimensionless"""
kPhotonvisionCameraDiagonalFOV = 75.1
"""degrees"""
kPhotonvisionCameraPixelDimensions = (960, 720)
kApriltagWidth = 8.125 * kMetersPerInch
kApriltagHeight = kApriltagWidth

# IntakeMotor Motors
kIntakeCANID = 3
kIntakePIDSlot = 0
kIntakePGain = 0.0001
kIntakeIGain = 0
kIntakeDGain = 0
kIntakeMotorPercent = 0.7
"""70%"""

kCubeLoadedKey = "intake/cubeInPos"
kConeLoadedKey = "intake/coneInPos"
kIntakeHoldingPercent = 0.0
"""5%"""

kIntakeStateKey = "intake/state"
kIntakeMotorRPMKey = "intake/RPM"
kIntakeGearRatio = 15
kCubeModeKey = "intake/CubeMode"
kIntakeMotorAMPS = 30
kPitchMultiplicationValue = 0.2

kDriveDistanceAutoBalance = 5

kSpeedFactorAutoBalance = 0.1

kTiltThresholdAutoBalance = 10
