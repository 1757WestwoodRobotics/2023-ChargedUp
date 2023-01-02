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
    Pose2d,
    Rotation2d,
    Translation2d,
    Pose3d,
    Rotation3d,
    Quaternion,
    Transform3d,
)
from wpimath.system.plant import DCMotor

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
kFieldLength = 52.5 * kMetersPerFoot
"""meters"""

kFieldWidth = 27 * kMetersPerFoot
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
    Pose3d(0.258, 0, 1.01, Rotation3d()),
)

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
kDrivePGain = 0.12
kDriveIGain = 0.0
kDriveDGain = 0.0

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

kRobotVisionPoseWeight = 0.05  # 19% vision data

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

kApriltagPositionDict = {  # this is gathered from a mapping of the field, derived with positions and ZYX order rotations converted to Quaternions in bulk in a spreadsheet
    0: Pose3d(-0.0035306, 7.5789282, 0.8858504, Rotation3d(Quaternion(1, 0, 0, 0))),
    1: Pose3d(3.2327088, 5.486654, 1.7254728, Rotation3d(Quaternion(1, 0, 0, 0))),
    2: Pose3d(
        3.067812,
        5.3305202,
        1.3762228,
        Rotation3d(Quaternion(0.707106781186548, 0, 0, -0.707106781186548)),
    ),
    3: Pose3d(0.0039878, 5.058537, 0.80645, Rotation3d(Quaternion(1, 0, 0, 0))),
    4: Pose3d(0.0039878, 3.5124898, 0.80645, Rotation3d(Quaternion(1, 0, 0, 0))),
    5: Pose3d(
        0.1211072,
        1.7178274,
        0.8906002,
        Rotation3d(Quaternion(0.919650220405092, 0, 0, 0.392738427084574)),
    ),
    6: Pose3d(
        0.8733028,
        0.9412986,
        0.8906002,
        Rotation3d(Quaternion(0.919650220405092, 0, 0, 0.392738427084574)),
    ),
    7: Pose3d(
        1.6150844,
        0.1572514,
        0.8906002,
        Rotation3d(Quaternion(0.919650220405092, 0, 0, 0.392738427084574)),
    ),
    10: Pose3d(
        16.4627306,
        0.6506718,
        0.8858504,
        Rotation3d(Quaternion(6.12323399573677e-17, 0, 0, 1)),
    ),
    11: Pose3d(
        13.2350002,
        2.743454,
        1.7254728,
        Rotation3d(Quaternion(6.12323399573677e-17, 0, 0, 1)),
    ),
    12: Pose3d(
        13.391388,
        2.8998418,
        1.3762228,
        Rotation3d(Quaternion(0.707106781186548, 0, 0, 0.707106781186548)),
    ),
    13: Pose3d(
        16.4552122,
        3.175508,
        0.80645,
        Rotation3d(Quaternion(6.12323399573677e-17, 0, 0, 1)),
    ),
    14: Pose3d(
        16.4552122,
        4.7171356,
        0.80645,
        Rotation3d(Quaternion(6.12323399573677e-17, 0, 0, 1)),
    ),
    15: Pose3d(
        16.3350194,
        6.514973,
        0.8937752,
        Rotation3d(Quaternion(-0.372987782575809, 0, 0, 0.92783625389892)),
    ),
    16: Pose3d(
        15.5904946,
        7.2926956,
        0.8906002,
        Rotation3d(Quaternion(-0.372987782575809, 0, 0, 0.92783625389892)),
    ),
    17: Pose3d(
        14.847189,
        8.0691228,
        0.8906002,
        Rotation3d(Quaternion(-0.372987782575809, 0, 0, 0.92783625389892)),
    ),
    40: Pose3d(
        7.874127,
        4.9131728,
        0.7032752,
        Rotation3d(Quaternion(0.544639035015027, 0, 0, 0.838670567945424)),
    ),
    41: Pose3d(
        7.4312272,
        3.759327,
        0.7032752,
        Rotation3d(Quaternion(-0.207911690817759, 0, 0, 0.978147600733806)),
    ),
    42: Pose3d(
        8.585073,
        3.3164272,
        0.7032752,
        Rotation3d(Quaternion(0.838670567945424, 0, 0, -0.544639035015027)),
    ),
    43: Pose3d(
        9.0279728,
        4.470273,
        0.7032752,
        Rotation3d(Quaternion(0.978147600733806, 0, 0, 0.207911690817759)),
    ),
    50: Pose3d(
        7.6790296,
        4.3261534,
        2.4177244,
        Rotation3d(
            Quaternion(
                0.177292733967826,
                -0.227449895715119,
                0.0421553464416173,
                0.9565859910054,
            )
        ),
    ),
    51: Pose3d(
        8.0182466,
        3.5642296,
        2.4177244,
        Rotation3d(
            Quaternion(
                -0.551043546584219,
                -0.19063969497247,
                -0.131023032308198,
                0.801773335471724,
            )
        ),
    ),
    52: Pose3d(
        8.7801704,
        3.9034466,
        2.4177244,
        Rotation3d(
            Quaternion(
                -0.956585991005399,
                -0.0421553464416174,
                -0.227449895715119,
                0.177292733967826,
            )
        ),
    ),
    53: Pose3d(
        8.4409534,
        4.6653704,
        2.4177244,
        Rotation3d(
            Quaternion(
                0.801773335471724,
                -0.131023032308198,
                0.19063969497247,
                0.551043546584219,
            )
        ),
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
kTrajectoryPositionPGain = 2.0
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 2.5
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
kNormalSpeedMultiplier = 0.45  # half full on normal
kTurboSpeedMultiplier = 0.90  # full speed!!!

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


kSimFrontLeftDriveEncoderPorts = (0, 1)
kSimFrontLeftSteerEncoderPorts = (2, 3)
kSimFrontRightDriveEncoderPorts = (4, 5)
kSimFrontRightSteerEncoderPorts = (6, 7)
kSimBackLeftDriveEncoderPorts = (8, 9)
kSimBackLeftSteerEncoderPorts = (10, 11)
kSimBackRightDriveEncoderPorts = (12, 13)
kSimBackRightSteerEncoderPorts = (14, 15)


kMotorBaseKey = "motors"

# Logging
kSwerveActualStatesKey = "swerve/actual"
kSwerveExpectedStatesKey = "swerve/expected"
kConsoleLog = "log"
kPDHCanID = 1
kPDHPublishKey = "powerDistribution"

kJoystickKeyLogPrefix = "DriverStation"
kFieldSimTargetKey = "SimTargets"
kFieldRelativeTargets = "RelTargets"

# photonvision parameters
kPhotonvisionCameraName = "cam"
kPhotonvisionAmbiguityCutoff = 0.2
"""dimensionless"""
kPhotonvisionCameraDiagonalFOV = 75.1
"""degrees"""
kPhotonvisionCameraPixelDimensions = (960, 720)
kApriltagWidth = 8.125 * kMetersPerInch
kApriltagHeight = kApriltagWidth
