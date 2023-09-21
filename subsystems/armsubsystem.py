import math
from enum import Enum, auto
from typing import Tuple
from commands2 import SubsystemBase
from wpilib import (
    Color,
    Color8Bit,
    Mechanism2d,
    Preferences,
    SmartDashboard,
)
import wpilib
from wpilib import DigitalInput, RobotState, Timer
from wpimath import angleModulus
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import (
    ProfiledPIDController,
    ArmFeedforward,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation2d,
    Translation3d,
)
from util.advantagescopeconvert import convertToSendablePoses
from util.angleoptimize import optimizeAngle
from util.convenientmath import clamp, pose3dFrom2d
from util.simcoder import CTREEncoder

from util.simfalcon import Falcon

import constants


# pylint: disable-next=too-many-instance-attributes
class ArmSubsystem(SubsystemBase):
    class ArmState(Enum):
        Stored = auto()
        Mid = auto()
        DoubleSubstation = auto()
        SingleSubtation = auto()
        Top = auto()
        GroundLoading = auto()
        GroundSafe = auto()
        GroundCone = auto()
        TopSafe = auto()
        OverrideValue = auto()
        Yoshi = auto()

        def position(self) -> Pose2d:
            if self == ArmSubsystem.ArmState.Stored:
                return constants.kArmStoredPosition
            elif self == ArmSubsystem.ArmState.Mid:
                return (
                    constants.kArmMidScorePositionCube
                    if SmartDashboard.getBoolean(constants.kCubeModeKey, False)
                    else (
                        (
                            constants.kArmMidScorePositionConeHook
                            if SmartDashboard.getBoolean(
                                constants.kFlangeHookKey, False
                            )
                            else constants.kArmMidScorePositionCube
                        )
                        if SmartDashboard.getBoolean(constants.kFlangeModeKey, False)
                        else constants.kArmMidScorePositionCone
                    )
                )
            elif self == ArmSubsystem.ArmState.DoubleSubstation:
                return (
                    constants.kArmDoubleSubstationPositionCube
                    if not SmartDashboard.getBoolean(constants.kCubeModeKey, False)
                    else constants.kArmDoubleSubstationPositionCone
                )
            elif self == ArmSubsystem.ArmState.SingleSubtation:
                return constants.kArmSingleSubstationPosition
            elif self == ArmSubsystem.ArmState.Top:
                return (
                    constants.kArmTopScorePositionCube
                    if SmartDashboard.getBoolean(constants.kCubeModeKey, False)
                    else (
                        (
                            constants.kArmTopScorePositionConeHook
                            if SmartDashboard.getBoolean(
                                constants.kFlangeHookKey, False
                            )
                            else constants.kArmTopScorePositionCube
                        )
                        if SmartDashboard.getBoolean(constants.kFlangeModeKey, False)
                        else constants.kArmTopScorePositionCone
                    )
                )
            elif self == ArmSubsystem.ArmState.GroundLoading:
                return (
                    constants.kArmGroundIntakePositionCube
                    if SmartDashboard.getBoolean(constants.kCubeModeKey, False)
                    else constants.kArmGroundIntakePositionCone
                )
            elif self == ArmSubsystem.ArmState.TopSafe:
                return constants.kArmTopSafePosition
            elif self == ArmSubsystem.ArmState.GroundCone:
                return constants.kArmGroundConeIntakePosition
            elif self == ArmSubsystem.ArmState.GroundSafe:
                return constants.kArmGroundSafePosition
            elif self == ArmSubsystem.ArmState.Yoshi:
                return constants.kArmGroundIntakePositionCubeYoshi
            return constants.kArmStoredPosition

        def oscilate(self) -> bool:
            # if self == ArmSubsystem.ArmState.GroundLoading:
            #     return True
            return False

    class InterpolationMethod(Enum):
        JointSpace = auto()
        CartesianSpace = auto()
        NoInterp = auto()

    def initMechanism(self) -> None:
        self.mech = Mechanism2d(90, 90)
        midNodeHome = self.mech.getRoot("Mid Node", 27.83, 0)
        self.midNode = midNodeHome.appendLigament(
            "Mid Cone Node", 34, 90, 10, Color8Bit(Color.kWhite)
        )

        highNodeHome = self.mech.getRoot("High Node", 10.58, 0)
        self.highNode = highNodeHome.appendLigament(
            "High Cone Node", 46, 90, 10, Color8Bit(Color.kWhite)
        )

        gridHome = self.mech.getRoot("Grid Home", 49.75, 0)
        self.gridNode = gridHome.appendLigament(
            "Grid Wall", 49.75, 180, 50, Color8Bit(Color.kWhite)
        )

        dsHome = self.mech.getRoot("Double Substation Home", 49.75, 37)
        self.dsRamp = dsHome.appendLigament(
            "Double Substation Ramp", 13.75, 180, 10, Color8Bit(Color.kWhite)
        )

        armPivot = self.mech.getRoot("ArmPivot", 56.83, 10)
        self.armTower = armPivot.appendLigament(
            "ArmTower", 0, -90, 10, Color8Bit(Color.kSilver)
        )

        self.armShoulder = armPivot.appendLigament(
            "Arm Shoulder",
            constants.kArmShoulderLength / constants.kMetersPerInch,
            -90,
            10,
            Color8Bit(Color.kGold),
        )
        self.armElbow = self.armShoulder.appendLigament(
            "Arm Elbow",
            constants.kArmElbowLength / constants.kMetersPerInch,
            45,
            10,
            Color8Bit(Color.kPurple),
        )
        self.armWrist = self.armElbow.appendLigament(
            "Wrist",
            constants.kArmWristLength / constants.kMetersPerInch,
            10,
            10,
            Color8Bit(Color.kWhite),
        )

        self.armShoulderEncoder = armPivot.appendLigament(
            "Arm Shoulder Encoder",
            constants.kArmShoulderLength / constants.kMetersPerInch,
            -90,
            10,
            Color8Bit(Color.kMediumAquamarine),
        )
        self.armElbowEncoder = self.armShoulderEncoder.appendLigament(
            "Arm Elbow Encoder",
            constants.kArmElbowLength / constants.kMetersPerInch,
            45,
            10,
            Color8Bit(Color.kAquamarine),
        )
        self.armWristEncoder = self.armElbowEncoder.appendLigament(
            "Wrist Encoder",
            constants.kArmWristLength / constants.kMetersPerInch,
            10,
            10,
            Color8Bit(Color.kAqua),
        )

        SmartDashboard.putData("Arm Sim", self.mech)

    def initEncoders(self) -> None:
        self.elbowEncoder = CTREEncoder(
            constants.kElbowArmCANCoderID,
            constants.kElbowArmCANCoderOffset,
            constants.kCANivoreName,
        )

        self.shoulderEncoder = CTREEncoder(
            constants.kShoulderArmCANCoderID,
            constants.kShoulderArmCANCoderOffset,
            constants.kCANivoreName,
        )

        self.wristEncoder = CTREEncoder(
            constants.kWristArmCANCoderID,
            constants.kWristArmCANCoderOffset,
            constants.kCANivoreName,
        )

    def _getShoulderEncoder(self) -> Rotation2d:
        return Rotation2d(
            angleModulus(-self.shoulderEncoder.getPosition().radians())
            / constants.kArmEncoderToSprocketGearRatio
        )

    def _getElbowEncoder(self) -> Rotation2d:
        return Rotation2d(
            angleModulus(self.elbowEncoder.getPosition().radians())
            / constants.kArmEncoderToSprocketGearRatio
        )

    def _getWristEncoder(self) -> Rotation2d:
        return Rotation2d(
            angleModulus(-self.wristEncoder.getPosition().radians())
            / constants.kArmEncoderToSprocketGearRatio
        )

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.initMechanism()

        self.state = ArmSubsystem.ArmState.Stored
        self.oldState = ArmSubsystem.ArmState.Stored
        self.armFF = ArmFeedforward(0, constants.kShoulderArmFFFactor, 0, 0)
        self.fudgeFactor = 0  # amount by which to adjust the wrist angle

        self.targetTimer = Timer()

        SmartDashboard.putNumber("arm/wristAdjust", 0)
        SmartDashboard.putNumber(constants.kElbowArmOverrideKey, 0)
        SmartDashboard.putNumber(constants.kShoulderArmOverrideKey, 0)
        SmartDashboard.putNumber(constants.kWristArmOverrideKey, 0)

        self.elbowArm = Falcon(
            constants.kElbowArmCANId,
            constants.kArmPIDSlot,
            constants.kElbowArmPGain,
            constants.kElbowArmIGain,
            constants.kElbowArmDGain,
            constants.kElbowArmInverted,
            canbus=constants.kCANivoreName,
            useDINSim=False,
        )
        self.elbowArm.setNeutralMode(Falcon.NeutralMode.Break)
        self.elbowArm.setCurrentLimit(constants.kDriveSupplyCurrentLimitConfiguration)

        self.shoulderArm = Falcon(
            constants.kShoulderArmCANId,
            constants.kArmPIDSlot,
            constants.kShoulderArmPGain,
            constants.kShoulderArmIGain,
            constants.kShoulderArmDGain,
            constants.kShoulderArmInverted,
            canbus=constants.kCANivoreName,
            useDINSim=False,
        )
        self.shoulderArm.setNeutralMode(Falcon.NeutralMode.Break)
        self.shoulderArm.setCurrentLimit(
            constants.kDriveSupplyCurrentLimitConfiguration
        )

        self.wristArm = Falcon(
            constants.kWristArmCANId,
            constants.kArmPIDSlot,
            constants.kWristArmPGain,
            constants.kWristArmIGain,
            constants.kWristArmDGain,
            constants.kWristArmInverted,
            canbus=constants.kCANivoreName,
            useDINSim=False,
        )
        self.wristArm.setNeutralMode(Falcon.NeutralMode.Break)
        self.wristArm.setCurrentLimit(constants.kDriveSupplyCurrentLimitConfiguration)

        self.xProfiledPID = ProfiledPIDController(
            constants.kArmTranslationalPGain,
            constants.kArmTranslationalIGain,
            constants.kArmTranslationalDGain,
            TrapezoidProfile.Constraints(
                constants.kArmTranslationalMaxVelocity,
                constants.kArmTranslationalMaxAcceleration,
            ),
        )
        self.yProfiledPID = ProfiledPIDController(
            constants.kArmTranslationalPGain,
            constants.kArmTranslationalIGain,
            constants.kArmTranslationalDGain,
            TrapezoidProfile.Constraints(
                constants.kArmTranslationalMaxVelocity,
                constants.kArmTranslationalMaxAcceleration,
            ),
        )
        self.thetaProfiledPID = ProfiledPIDControllerRadians(
            constants.kArmRotationalPGain,
            constants.kArmRotationalIGain,
            constants.kArmRotationalDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kArmRotationalMaxVelocity,
                constants.kArmRotationalMaxAcceleration,
            ),
        )

        self.shoulderPID = ProfiledPIDControllerRadians(
            constants.kArmRotationalPGain,
            constants.kArmRotationalIGain,
            constants.kArmRotationalDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kArmRotationalMaxVelocity,
                constants.kArmRotationalMaxAcceleration,
            ),
        )
        self.shoulderPID.enableContinuousInput(-math.pi, math.pi)
        self.elbowPID = ProfiledPIDControllerRadians(
            constants.kArmRotationalPGain,
            constants.kArmRotationalIGain,
            constants.kArmRotationalDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kArmRotationalMaxVelocity,
                constants.kArmRotationalMaxAcceleration,
            ),
        )
        self.elbowPID.enableContinuousInput(-math.pi, math.pi)

        self.xProfiledPID.setTolerance(constants.kArmPositionTolerence)
        self.yProfiledPID.setTolerance(constants.kArmPositionTolerence)
        self.thetaProfiledPID.setTolerance(constants.kArmRotationTolerence)
        self.elbowPID.setTolerance(constants.kArmRotationTolerence)
        self.shoulderPID.setTolerance(constants.kArmRotationTolerence)

        self.interpolationMethod = wpilib.SendableChooser()
        self.interpolationMethod.addOption(
            "Joint Space", ArmSubsystem.InterpolationMethod.JointSpace
        )
        self.interpolationMethod.setDefaultOption(
            "Cartesian Space", ArmSubsystem.InterpolationMethod.CartesianSpace
        )
        self.interpolationMethod.addOption(
            "No Interpolation", ArmSubsystem.InterpolationMethod.NoInterp
        )
        SmartDashboard.putData(
            constants.kArmInterpolationMethod, self.interpolationMethod
        )

        self.motorMode = wpilib.SendableChooser()
        self.motorMode.setDefaultOption("Brake Mode", Falcon.NeutralMode.Break)
        self.motorMode.addOption("Coast Mode", Falcon.NeutralMode.Coast)

        SmartDashboard.putData(constants.kArmMotorBreakArmModeKey, self.motorMode)

        self.targetPose = Pose2d()
        self.targetElbow = Pose2d()

        self.totalCOM = Translation2d()
        self.elbowRelativeCOM = Translation2d()
        self.wristRelativeCOM = Translation2d()

        self.expectedWrist = 0.0  # radians
        self.expectedTwoLink = Translation2d()
        self.initEncoders()
        self.reset()
        Preferences.initBoolean(constants.kArmObeyEndstopsKey, True)

        self.inputButton = DigitalInput(0)

    def reset(self) -> None:
        pose = constants.kArmStartupPosition
        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmWristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmWristLength * pose.rotation().sin(),
        )
        shoulderAngle, elbowAngle, wristAngle = self._armAnglesAtPosiiton(
            Pose2d(twoLinkPosition, pose.rotation())
        )

        shoulderAngle, elbowAngle, wristAngle = constants.kArmStartupAngles
        # shoulderOffset = self._getShoulderEncoder().radians()
        # elbowOffset = self._getElbowEncoder().radians()
        # wristOffset = self._getWristEncoder().radians()
        shoulderOffset = 0
        elbowOffset = 0
        wristOffset = 0

        self.shoulderArm.setEncoderPosition(
            (shoulderAngle + shoulderOffset)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kShoulderArmGearRatio
        )
        self.elbowArm.setEncoderPosition(
            (shoulderAngle + elbowAngle + elbowOffset)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kElbowArmGearRatio
        )
        self.wristArm.setEncoderPosition(
            (shoulderAngle + elbowAngle + wristAngle + wristOffset)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristArmGearRatio
        )

        self.expectedTwoLink = twoLinkPosition
        self.expectedWrist = pose.rotation().radians()

        self.xProfiledPID.reset(twoLinkPosition.X())
        self.yProfiledPID.reset(twoLinkPosition.Y())
        self.thetaProfiledPID.reset(pose.rotation().radians())

    def _getShoulderRawArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.shoulderArm.get(Falcon.ControlMode.Position)
            / constants.kShoulderArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        )

    def getShoulderArmRotation(self) -> Rotation2d:
        return optimizeAngle(Rotation2d(), self._getShoulderRawArmRotation())

    def _getElbowRawArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.elbowArm.get(Falcon.ControlMode.Position)
            / constants.kElbowArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        )

    def getElbowArmRotation(self) -> Rotation2d:
        return (
            optimizeAngle(Rotation2d(), self._getElbowRawArmRotation())
            - self.getShoulderArmRotation()
        )  # 4 bar to relative

    def _getWristRawArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.wristArm.get(Falcon.ControlMode.Position)
            / constants.kWristArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        ) + Rotation2d.fromDegrees(SmartDashboard.getNumber("arm/wristAdjust", 0))

    def getWristArmRotation(self) -> Rotation2d:
        return (
            optimizeAngle(Rotation2d(), self._getWristRawArmRotation())
            - self.getShoulderArmRotation()
            - self.getElbowArmRotation()
        )  # 4 bar to relative

    def getElbowPosition(self) -> Pose2d:
        shoulderRot = self.getShoulderArmRotation()
        return (
            Pose2d()
            + Transform2d(Translation2d(), shoulderRot)
            + Transform2d(Translation2d(constants.kArmShoulderLength, 0), Rotation2d())
        )

    def getWristPosition(self) -> Pose2d:
        elbowPosition = self.getElbowPosition()
        elbowRot = self.getElbowArmRotation()
        return (
            elbowPosition
            + Transform2d(Translation2d(), elbowRot)
            + Transform2d(Translation2d(constants.kArmElbowLength, 0), Rotation2d())
        )

    def getEndEffectorPosition(self) -> Pose2d:
        wristPosition = self.getWristPosition()
        wristRot = self.getWristArmRotation()
        return (
            wristPosition
            + Transform2d(Translation2d(), wristRot)
            + Transform2d(Translation2d(constants.kArmWristLength, 0), Rotation2d())
        )

    def _updateCOMs(self) -> None:
        shoulderRotation = self.getShoulderArmRotation()
        elbowRotation = self.getElbowArmRotation() + shoulderRotation
        wristRotation = self.getWristArmRotation() + elbowRotation
        # these rotations are relative to the floor

        elbowPose = self.getElbowPosition()
        wristPose = self.getWristPosition()

        armCOM = Translation2d(constants.kArmShoulderCOM, shoulderRotation)
        forearmCOM = elbowPose.translation() + Translation2d(
            constants.kArmElbowCOM, elbowRotation
        )
        handCOM = wristPose.translation() + Translation2d(
            constants.kArmWristCOM, wristRotation
        )

        self.totalCOM = Translation2d(
            (
                armCOM.X() * constants.kArmShoulderMass
                + forearmCOM.X() * constants.kArmElbowMass
                + handCOM.X() * constants.kArmWristMass
            )
            / (
                constants.kArmShoulderMass
                + constants.kArmElbowMass
                + constants.kArmWristMass
            ),
            (
                armCOM.Y() * constants.kArmShoulderMass
                + forearmCOM.Y() * constants.kArmElbowMass
                + handCOM.Y() * constants.kArmWristMass
            )
            / (
                constants.kArmShoulderMass
                + constants.kArmElbowMass
                + constants.kArmWristMass
            ),
        )

        self.elbowRelativeCOM = Translation2d(
            (
                forearmCOM.X() * constants.kArmElbowMass
                + handCOM.X() * constants.kArmWristMass
            )
            / (constants.kArmElbowMass + constants.kArmWristMass),
            (
                forearmCOM.Y() * constants.kArmElbowMass
                + handCOM.Y() * constants.kArmWristMass
            )
            / (constants.kArmElbowMass + constants.kArmWristMass),
        )

        self.wristRelativeCOM = handCOM

    def _shoulderFF(self, globalRelativeRotation: Rotation2d) -> float:
        shoulderRelativeRadius = self.totalCOM.norm()

        ffAmount = (
            globalRelativeRotation.cos()
            * shoulderRelativeRadius
            * constants.kShoulderArmFFFactor
        )
        SmartDashboard.putNumber(constants.kShoulderFeedForwardLogKey, ffAmount)
        return ffAmount

    def _elbowFF(self, globalRelativeRotation: Rotation2d) -> float:
        elbowRelativeRadius = self.elbowRelativeCOM.distance(
            self.getElbowPosition().translation()
        )
        ffAmount = (
            globalRelativeRotation.cos()
            * elbowRelativeRadius
            * constants.kElbowArmFFFactor
        )
        SmartDashboard.putNumber(constants.kElbowFeedForwardLogKey, ffAmount)
        return ffAmount

    def _wristFF(self, globalRelativeRotation: Rotation2d) -> float:
        wristRelativeRadius = self.wristRelativeCOM.distance(
            self.getWristPosition().translation()
        )
        ffAmount = (
            globalRelativeRotation.cos()
            * wristRelativeRadius
            * constants.kWristArmFFFactor
        )
        SmartDashboard.putNumber(constants.kWristFeedForwardLogKey, ffAmount)
        return ffAmount

    def _updateArmPositionsLogging(self) -> None:
        robotPose3d = pose3dFrom2d(
            Pose2d(
                *SmartDashboard.getNumberArray(
                    constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
                )
            )
        )
        shoulderPose = (
            robotPose3d
            + constants.kShoulderRobotOffset
            + Transform3d(
                Translation3d(),
                Rotation3d(0, self.getShoulderArmRotation().radians(), 0),
            )
        )
        elbowPose = shoulderPose + Transform3d(
            Translation3d(constants.kArmShoulderLength, 0, 0),
            Rotation3d(0, self.getElbowArmRotation().radians(), 0),
        )
        wristPose = elbowPose + Transform3d(
            Translation3d(constants.kArmElbowLength, 0, 0),
            Rotation3d(0, self.getWristArmRotation().radians(), 0),
        )
        endEffectorPose = wristPose + Transform3d(
            Translation3d(constants.kArmWristLength, 0, 0), Rotation3d()
        )

        targetPose = (
            robotPose3d
            + constants.kShoulderRobotOffset
            + Transform3d(
                Translation3d(self.targetPose.X(), 0, -self.targetPose.Y()),
                Rotation3d(0, self.targetPose.rotation().radians(), 0),
            )
        )
        targetElbow = (
            robotPose3d
            + constants.kShoulderRobotOffset
            + Transform3d(
                Translation3d(self.targetElbow.X(), 0, -self.targetElbow.Y()),
                Rotation3d(0, self.targetElbow.rotation().radians(), 0),
            )
        )

        targetTarget = (
            robotPose3d
            + constants.kShoulderRobotOffset
            + Transform3d(
                Translation3d(self.expectedTwoLink.X(), 0, -self.expectedTwoLink.Y()),
                Rotation3d(0, self.expectedWrist, 0),
            )
        )
        armPosesSerialized = convertToSendablePoses(
            [
                shoulderPose,
                elbowPose,
                wristPose,
                endEffectorPose,
            ]
        )

        targetPosesSerialized = convertToSendablePoses([targetTarget])

        comsSerialized = convertToSendablePoses(
            list(
                map(
                    lambda x: robotPose3d
                    + Transform3d(Translation3d(-x.X(), 0, x.Y()), Rotation3d())
                    + constants.kShoulderRobotOffset,
                    [self.totalCOM, self.elbowRelativeCOM, self.wristRelativeCOM],
                )
            )
        )

        SmartDashboard.putNumberArray(constants.kArmPosesKey, armPosesSerialized)
        SmartDashboard.putNumberArray(
            constants.kArmTargetPosesKey, targetPosesSerialized
        )
        SmartDashboard.putNumberArray(constants.kArmCOMs, comsSerialized)

    def _updateMechanism(self) -> None:
        self.armElbow.setAngle(self.getElbowArmRotation().degrees())
        self.armShoulder.setAngle(self.getShoulderArmRotation().degrees())
        self.armWrist.setAngle(self.getWristArmRotation().degrees())

        pose = constants.kArmStartupPosition
        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmWristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmWristLength * pose.rotation().sin(),
        )
        shoulderAngle, elbowAngle, wristAngle = self._armAnglesAtPosiiton(
            Pose2d(twoLinkPosition, pose.rotation())
        )

        self.armElbowEncoder.setAngle(
            self._getElbowEncoder().degrees()
            + elbowAngle / constants.kRadiansPerDegree
            - self._getShoulderEncoder().degrees()
        )
        self.armShoulderEncoder.setAngle(
            self._getShoulderEncoder().degrees()
            + shoulderAngle / constants.kRadiansPerDegree
        )
        self.armWristEncoder.setAngle(
            self._getWristEncoder().degrees()
            + wristAngle / constants.kRadiansPerDegree
            - self._getElbowEncoder().degrees()
        )

        endEffectorPose = self.getEndEffectorPosition()
        SmartDashboard.putNumberArray(
            constants.kArmEndEffectorPose,
            [
                endEffectorPose.X() / constants.kMetersPerInch,
                endEffectorPose.Y() / constants.kMetersPerInch,
                endEffectorPose.rotation().degrees(),
            ],  # publish how it is in constants
        )

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kArmStateKey, str(self.state))

        shoulderRotation = self.getShoulderArmRotation()
        elbowRotation = self.getElbowArmRotation()
        wristRotation = self.getWristArmRotation()

        SmartDashboard.putNumber(
            constants.kElbowArmRotationKey, elbowRotation.degrees()
        )
        SmartDashboard.putNumber(
            constants.kShoulderArmRotationKey, shoulderRotation.degrees()
        )
        SmartDashboard.putNumber(
            constants.kWristArmRotationKey, wristRotation.degrees()
        )
        SmartDashboard.putBoolean(constants.kArmAtTargetKey, self.atTarget())

        SmartDashboard.putNumber(
            constants.kArmShoulderActualMotorKey, shoulderRotation.degrees()
        )
        SmartDashboard.putNumber(
            constants.kArmElbowActualMotorKey,
            optimizeAngle(
                Rotation2d.fromDegrees(200), (shoulderRotation + elbowRotation)
            ).degrees(),
        )
        SmartDashboard.putNumber(
            constants.kArmWristActualMotorKey,
            (wristRotation + shoulderRotation + elbowRotation).degrees(),
        )
        SmartDashboard.putNumber(constants.kArmFudgeFactorKey, self.fudgeFactor)

        SmartDashboard.putNumber("arm/targetThought/x", self.expectedTwoLink.X())
        SmartDashboard.putNumber("arm/targetThought/y", self.expectedTwoLink.Y())
        SmartDashboard.putNumber("arm/targetThought/theta", self.expectedWrist)

        motorNeutralState: Falcon.NeutralMode = self.motorMode.getSelected()

        if RobotState.isDisabled():
            motorNeutralState = (
                Falcon.NeutralMode.Break
                if self.inputButton.get()
                else Falcon.NeutralMode.Coast
            )
        self.shoulderArm.setNeutralMode(motorNeutralState)
        self.elbowArm.setNeutralMode(motorNeutralState)
        self.wristArm.setNeutralMode(motorNeutralState)

        if self.state == ArmSubsystem.ArmState.OverrideValue:
            elbow = SmartDashboard.getNumber(constants.kElbowArmOverrideKey, 0)
            shoulder = SmartDashboard.getNumber(constants.kShoulderArmOverrideKey, 0)
            wrist = SmartDashboard.getNumber(constants.kWristArmOverrideKey, 0)

            self.setRelativeArmAngles(
                Rotation2d.fromDegrees(shoulder),
                Rotation2d.fromDegrees(elbow),
                Rotation2d.fromDegrees(wrist),
            )
        else:
            targetState = self.state.position()
            self.setEndEffectorPosition(targetState)

        if self.state != self.oldState:
            self.oldState = self.state
            self.targetTimer.reset()
            self.targetTimer.start()

        self._updateMechanism()
        self._updateCOMs()
        self._updateArmPositionsLogging()

    def resetTimer(self) -> None:
        self.targetTimer.reset()
        self.targetTimer.start()

    def _canElbowReachPosition(self, position: Translation2d):
        return (
            position.distance(Translation2d())
            < constants.kArmShoulderLength + constants.kArmElbowLength
            and position.distance(Translation2d())
            > constants.kArmShoulderLength - constants.kArmElbowLength
        )

    def _nearestPossibleElbowPosition(self, position: Translation2d) -> Translation2d:
        dist = position.distance(Translation2d())
        if dist < constants.kArmShoulderLength - constants.kArmElbowLength:
            return Translation2d(
                position.X(), position.Y() + constants.kArmPositionExtraEpsiolon
            )
        elif dist > constants.kArmElbowLength + constants.kArmShoulderLength:
            return Translation2d(
                position.X(), position.Y() - constants.kArmPositionExtraEpsiolon
            )
        else:
            return position

    def atTarget(self) -> bool:
        return (
            (
                (
                    abs(
                        (
                            self.expectedTwoLink - self.getWristPosition().translation()
                        ).norm()
                    )
                    < (
                        constants.kArmPositionStoredTolerence
                        if self.state == ArmSubsystem.ArmState.Stored
                        else constants.kArmPositionTolerence
                    )
                )
                # or (self.elbowPID.atGoal() and self.shoulderPID.atGoal())
            )
            and (
                abs(
                    angleModulus(
                        self.expectedWrist
                        - self.getEndEffectorPosition().rotation().radians()
                    )
                )
                < constants.kArmRotationTolerence
            )
            and (self.targetTimer.get() > 0.06)
        )

    def _armAnglesAtPosiiton(self, pose: Pose2d) -> Tuple[float, float, float]:
        endAngle = math.acos(
            (
                pose.X() * pose.X()
                + pose.Y() * pose.Y()
                - constants.kArmElbowLength * constants.kArmElbowLength
                - constants.kArmShoulderLength * constants.kArmShoulderLength
            )
            / (2 * constants.kArmElbowLength * constants.kArmShoulderLength)
        )

        startAngle = math.atan2(pose.Y(), pose.X()) - math.atan2(
            math.sin(endAngle) * constants.kArmElbowLength,
            constants.kArmShoulderLength
            + math.cos(endAngle) * constants.kArmElbowLength,
        )

        wristAngle = pose.rotation().radians() - startAngle - endAngle

        return startAngle, endAngle, wristAngle

    def setFudgeFactor(self, factor: float) -> None:
        self.fudgeFactor = factor

    def setEndEffectorPosition(self, pose: Pose2d):
        desiredInterpolation: ArmSubsystem.InterpolationMethod = (
            self.interpolationMethod.getSelected()
        )

        self.targetPose = pose

        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmWristLength * pose.rotation().cos(),
            pose.Y()
            - constants.kArmWristLength * pose.rotation().sin()
            + self.fudgeFactor,
        )

        targetTwoLink = twoLinkPosition
        if desiredInterpolation == ArmSubsystem.InterpolationMethod.CartesianSpace:
            xDelta = self.xProfiledPID.calculate(
                self.expectedTwoLink.X(), twoLinkPosition.X()
            )
            yDelta = self.yProfiledPID.calculate(
                self.expectedTwoLink.Y(), twoLinkPosition.Y()
            )
            self.expectedTwoLink += Translation2d(xDelta, yDelta)

            targetTwoLink = self.expectedTwoLink

        while not self._canElbowReachPosition(targetTwoLink):
            targetTwoLink = self._nearestPossibleElbowPosition(targetTwoLink)

        startAngle, endAngle, wristAngle = self._armAnglesAtPosiiton(
            Pose2d(targetTwoLink, pose.rotation())
        )

        self.targetElbow = Pose2d(targetTwoLink, pose.rotation())

        if self.state.oscilate():
            wristAngle += (
                math.sin(Timer.getFPGATimestamp() * 10)
                * constants.kArmMaxOscillationAmount.radians()
                + constants.kArmMaxOscillationAmount.radians() / 2
            )

        self.setRelativeArmAngles(
            Rotation2d(startAngle),
            Rotation2d(endAngle),
            Rotation2d(wristAngle),
        )

    def setRelativeArmAngles(
        self, shoulder: Rotation2d, elbow: Rotation2d, wrist: Rotation2d
    ) -> None:
        SmartDashboard.putNumber(constants.kElbowArmTargetRotationKey, elbow.degrees())
        SmartDashboard.putNumber(
            constants.kShoulderTargetArmRotationKey, shoulder.degrees()
        )
        SmartDashboard.putNumber(constants.kWristTargetArmRotationKey, wrist.degrees())

        clampedShoulder = clamp(
            optimizeAngle(Rotation2d.fromDegrees(90), shoulder).radians(),
            constants.kShoulderMinAngle.radians(),
            constants.kShoulderMaxAngle.radians(),
        )
        clampedElbow = clamp(
            optimizeAngle(Rotation2d(), elbow).radians(),
            constants.kElbowMinAngle.radians(),
            constants.kElbowMaxAngle.radians(),
        )

        clampedWrist = clamp(
            optimizeAngle(Rotation2d.fromDegrees(-45), wrist).radians(),
            constants.kWristMinAngle.radians(),
            constants.kWristMaxAngle.radians(),
        )

        if not Preferences.getBoolean(constants.kArmObeyEndstopsKey, True):
            clampedShoulder = optimizeAngle(
                Rotation2d.fromDegrees(90), shoulder
            ).radians()
            clampedElbow = angleModulus(elbow.radians())
            clampedWrist = angleModulus(wrist.radians())

        trueShoulderPos = clampedShoulder
        trueElbowPos = clampedElbow + trueShoulderPos
        trueWristPos = clampedWrist + trueElbowPos

        desiredInterpolation: ArmSubsystem.InterpolationMethod = (
            self.interpolationMethod.getSelected()
        )

        if desiredInterpolation == ArmSubsystem.InterpolationMethod.JointSpace:
            currentElbowRaw = self._getWristRawArmRotation()
            currentShoulderRaw = self._getShoulderRawArmRotation()

            trueShoulderPos = (
                self.shoulderPID.calculate(
                    currentShoulderRaw.radians(), trueShoulderPos
                )
                + currentShoulderRaw.radians()
            )
            trueElbowPos = (
                self.elbowPID.calculate(currentElbowRaw.radians(), trueElbowPos)
                + currentElbowRaw.radians()
            )

        thetaDelta = self.thetaProfiledPID.calculate(self.expectedWrist, trueWristPos)
        trueWristPos = (
            thetaDelta
            + self.expectedWrist
            - SmartDashboard.getNumber("arm/wristAdjust", 0)
            * constants.kRadiansPerDegree
        )

        self.expectedWrist = trueWristPos

        SmartDashboard.putNumber(
            constants.kArmShoulderTargetMotorKey,
            trueShoulderPos / constants.kRadiansPerDegree,
        )
        SmartDashboard.putNumber(
            constants.kArmElbowTargetMotorKey,
            trueElbowPos / constants.kRadiansPerDegree,
        )
        SmartDashboard.putNumber(
            constants.kArmWristTargetMotorKey,
            trueWristPos / constants.kRadiansPerDegree,
        )

        shoulderArmEncoderPulses = (
            trueShoulderPos
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kShoulderArmGearRatio
        )
        elbowArmEncoderPulses = (
            trueElbowPos
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kElbowArmGearRatio
        )
        wristArmEncoderPulses = (
            trueWristPos
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristArmGearRatio
        )

        self.elbowArm.set(
            Falcon.ControlMode.Position,
            elbowArmEncoderPulses,
            self._elbowFF(Rotation2d(trueElbowPos)),
        )
        self.shoulderArm.set(
            Falcon.ControlMode.Position,
            shoulderArmEncoderPulses,
            self._shoulderFF(Rotation2d(trueShoulderPos)),
        )
        self.wristArm.set(
            Falcon.ControlMode.Position,
            wristArmEncoderPulses,
            self._wristFF(Rotation2d(trueWristPos)),
        )
