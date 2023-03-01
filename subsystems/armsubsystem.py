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

from util.simfalcon import Falcon

import constants


class ArmSubsystem(SubsystemBase):
    class ArmState(Enum):
        Stored = auto()
        Mid = auto()
        DoubleSubstation = auto()
        SingleSubtation = auto()
        Top = auto()
        GroundLoading = auto()
        GroundSafe = auto()
        OverrideValue = auto()

        def position(self) -> Pose2d:
            if self == ArmSubsystem.ArmState.Stored:
                return constants.kArmStoredPosition
            elif self == ArmSubsystem.ArmState.Mid:
                return constants.kArmMidScorePosition
            elif self == ArmSubsystem.ArmState.DoubleSubstation:
                return constants.kArmDoubleSubstationPosition
            elif self == ArmSubsystem.ArmState.SingleSubtation:
                return constants.kArmSingleSubstationPosition
            elif self == ArmSubsystem.ArmState.Top:
                return constants.kArmTopScorePosition
            elif self == ArmSubsystem.ArmState.GroundLoading:
                return constants.kArmGroundIntakePosition
            elif self == ArmSubsystem.ArmState.GroundSafe:
                return constants.kArmGroundSafePosition
            return constants.kArmStoredPosition

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
            constants.kArmshoulderLength / constants.kMetersPerInch,
            -90,
            10,
            Color8Bit(Color.kGold),
        )
        self.armElbow = self.armShoulder.appendLigament(
            "Arm Elbow",
            constants.kArmelbowLength / constants.kMetersPerInch,
            45,
            10,
            Color8Bit(Color.kPurple),
        )
        self.armWrist = self.armElbow.appendLigament(
            "Wrist",
            constants.kArmwristLength / constants.kMetersPerInch,
            10,
            10,
            Color8Bit(Color.kWhite),
        )

        SmartDashboard.putData("Arm Sim", self.mech)

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.initMechanism()

        self.state = ArmSubsystem.ArmState.Stored
        self.armFF = ArmFeedforward(0, constants.kShoulderArmFFFactor, 0, 0)

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
        self.elbowPID = ProfiledPIDControllerRadians(
            constants.kArmRotationalPGain,
            constants.kArmRotationalIGain,
            constants.kArmRotationalDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kArmRotationalMaxVelocity,
                constants.kArmRotationalMaxAcceleration,
            ),
        )

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

        self.targetPose = Pose2d()
        self.targetElbow = Pose2d()

        self.totalCOM = Translation2d()
        self.elbowRelativeCOM = Translation2d()
        self.wristRelativeCOM = Translation2d()

        self.reset()
        Preferences.initBoolean(constants.kArmObeyEndstopsKey, True)

    def reset(self) -> None:
        pose = constants.kArmStoredPosition
        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmwristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmwristLength * pose.rotation().sin(),
        )
        shoulderAngle, elbowAngle, wristAngle = self._armAnglesAtPosiiton(
            Pose2d(twoLinkPosition, pose.rotation())
        )

        self.shoulderArm.setEncoderPosition(
            shoulderAngle
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kShoulderArmGearRatio
        )
        self.elbowArm.setEncoderPosition(
            (shoulderAngle + elbowAngle)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kElbowArmGearRatio
        )
        self.wristArm.setEncoderPosition(
            (shoulderAngle + elbowAngle + wristAngle)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristArmGearRatio
        )

    def getElbowArmRotation(self) -> Rotation2d:
        return (
            Rotation2d(
                angleModulus(
                    self.elbowArm.get(Falcon.ControlMode.Position)
                    / constants.kElbowArmGearRatio
                    / constants.kTalonEncoderPulsesPerRadian
                )
            )
            - self.getShoulderArmRotation()
        )  # 4 bar to relative

    def getShoulderArmRotation(self) -> Rotation2d:
        return Rotation2d(
            angleModulus(
                self.shoulderArm.get(Falcon.ControlMode.Position)
                / constants.kShoulderArmGearRatio
                / constants.kTalonEncoderPulsesPerRadian
            )
        )

    def getWristArmRotation(self) -> Rotation2d:
        return (
            Rotation2d(
                angleModulus(
                    self.wristArm.get(Falcon.ControlMode.Position)
                    / constants.kWristArmGearRatio
                    / constants.kTalonEncoderPulsesPerRadian
                )
            )
            - self.getShoulderArmRotation()
            - self.getElbowArmRotation()
        )  # 4 bar to relative

    def getElbowPosition(self) -> Pose2d:
        shoulderRot = self.getShoulderArmRotation()
        return (
            Pose2d()
            + Transform2d(Translation2d(), shoulderRot)
            + Transform2d(Translation2d(constants.kArmshoulderLength, 0), Rotation2d())
        )

    def getWristPosition(self) -> Pose2d:
        elbowPosition = self.getElbowPosition()
        elbowRot = self.getElbowArmRotation()
        return (
            elbowPosition
            + Transform2d(Translation2d(), elbowRot)
            + Transform2d(Translation2d(constants.kArmelbowLength, 0), Rotation2d())
        )

    def getEndEffectorPosition(self) -> Pose2d:
        wristPosition = self.getWristPosition()
        wristRot = self.getWristArmRotation()
        return (
            wristPosition
            + Transform2d(Translation2d(), wristRot)
            + Transform2d(Translation2d(constants.kArmwristLength, 0), Rotation2d())
        )

    def _updateCOMs(self) -> None:
        shoulderRotation = self.getShoulderArmRotation()
        elbowRotation = self.getElbowArmRotation() + shoulderRotation
        wristRotation = self.getWristArmRotation() + elbowRotation
        # these rotations are relative to the floor

        elbowPose = self.getElbowPosition()
        wristPose = self.getWristPosition()

        armCOM = Translation2d(constants.kArmshoulderCOM, shoulderRotation)
        forearmCOM = elbowPose.translation() + Translation2d(
            constants.kArmelbowCOM, elbowRotation
        )
        handCOM = wristPose.translation() + Translation2d(
            constants.kArmWristCOM, wristRotation
        )

        self.totalCOM = Translation2d(
            (
                armCOM.X() * constants.kArmshoulderMass
                + forearmCOM.X() * constants.kArmelbowMass
                + handCOM.X() * constants.kArmwristMass
            )
            / (
                constants.kArmshoulderMass
                + constants.kArmelbowMass
                + constants.kArmwristMass
            ),
            (
                armCOM.Y() * constants.kArmshoulderMass
                + forearmCOM.Y() * constants.kArmelbowMass
                + handCOM.Y() * constants.kArmwristMass
            )
            / (
                constants.kArmshoulderMass
                + constants.kArmelbowMass
                + constants.kArmwristMass
            ),
        )

        self.elbowRelativeCOM = Translation2d(
            (
                forearmCOM.X() * constants.kArmelbowMass
                + handCOM.X() * constants.kArmwristMass
            )
            / (constants.kArmelbowMass + constants.kArmwristMass),
            (
                forearmCOM.Y() * constants.kArmelbowMass
                + handCOM.Y() * constants.kArmwristMass
            )
            / (constants.kArmelbowMass + constants.kArmwristMass),
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
            Translation3d(constants.kArmshoulderLength, 0, 0),
            Rotation3d(0, self.getElbowArmRotation().radians(), 0),
        )
        wristPose = elbowPose + Transform3d(
            Translation3d(constants.kArmelbowLength, 0, 0),
            Rotation3d(0, self.getWristArmRotation().radians(), 0),
        )
        endEffectorPose = wristPose + Transform3d(
            Translation3d(constants.kArmwristLength, 0, 0), Rotation3d()
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
        armPosesSerialized = convertToSendablePoses(
            [
                shoulderPose,
                elbowPose,
                wristPose,
                endEffectorPose,
            ]
        )

        targetPosesSerialized = convertToSendablePoses(
            [
                targetPose,
                targetElbow,
            ]
        )

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
        if self.state == ArmSubsystem.ArmState.OverrideValue:
            elbow = SmartDashboard.getNumber(constants.kElbowArmOverrideKey, 0)
            shoulder = SmartDashboard.getNumber(constants.kShoulderArmOverrideKey, 0)
            wrist = SmartDashboard.getNumber(constants.kWristArmOverrideKey, 0)

            self.setRelativeArmAngles(
                Rotation2d.fromDegrees(elbow),
                Rotation2d.fromDegrees(shoulder),
                Rotation2d.fromDegrees(wrist),
            )
        else:
            targetState = self.state.position()
            self.setEndEffectorPosition(targetState)

        self._updateMechanism()
        self._updateCOMs()
        self._updateArmPositionsLogging()

    def _canElbowReachPosition(self, position: Translation2d):
        return (
            position.distance(Translation2d())
            < constants.kArmshoulderLength + constants.kArmelbowLength
            and position.distance(Translation2d())
            > constants.kArmshoulderLength - constants.kArmelbowLength
        )

    def _nearestPossibleElbowPosition(self, position: Translation2d) -> Translation2d:
        dist = position.distance(Translation2d())
        if dist < constants.kArmshoulderLength - constants.kArmelbowLength:
            return Translation2d(
                position.X(), position.Y() + constants.kArmPositionExtraEpsiolon
            )
        elif dist > constants.kArmelbowLength + constants.kArmshoulderLength:
            return Translation2d(
                position.X(), position.Y() - constants.kArmPositionExtraEpsiolon
            )
        else:
            return position

    def atTarget(self) -> bool:
        return (
            (self.xProfiledPID.atGoal() and self.yProfiledPID.atGoal())
            or (self.elbowPID.atGoal() and self.shoulderPID.atGoal())
        ) and self.thetaProfiledPID.atGoal()

    def _armAnglesAtPosiiton(self, pose: Pose2d) -> Tuple[float, float, float]:
        endAngle = math.acos(
            (
                pose.X() * pose.X()
                + pose.Y() * pose.Y()
                - constants.kArmelbowLength * constants.kArmelbowLength
                - constants.kArmshoulderLength * constants.kArmshoulderLength
            )
            / (2 * constants.kArmelbowLength * constants.kArmshoulderLength)
        )

        startAngle = math.atan2(pose.Y(), pose.X()) - math.atan2(
            math.sin(endAngle) * constants.kArmelbowLength,
            constants.kArmshoulderLength
            + math.cos(endAngle) * constants.kArmelbowLength,
        )

        wristAngle = pose.rotation().radians() - startAngle - endAngle

        return startAngle, endAngle, wristAngle

    def setEndEffectorPosition(self, pose: Pose2d):
        desiredInterpolation: ArmSubsystem.InterpolationMethod = (
            self.interpolationMethod.getSelected()
        )

        self.targetPose = pose

        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmwristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmwristLength * pose.rotation().sin(),
        )

        targetTwoLink = twoLinkPosition
        if desiredInterpolation == ArmSubsystem.InterpolationMethod.CartesianSpace:
            currentWristPose = self.getWristPosition()
            targetTwoLink = Translation2d(
                self.xProfiledPID.calculate(currentWristPose.X(), twoLinkPosition.X())
                + currentWristPose.X(),
                self.yProfiledPID.calculate(currentWristPose.Y(), twoLinkPosition.Y())
                + currentWristPose.Y(),
            )
        while not self._canElbowReachPosition(targetTwoLink):
            targetTwoLink = self._nearestPossibleElbowPosition(targetTwoLink)

        startAngle, endAngle, wristAngle = self._armAnglesAtPosiiton(
            Pose2d(targetTwoLink, pose.rotation())
        )

        if desiredInterpolation == ArmSubsystem.InterpolationMethod.CartesianSpace:
            currentWristRotation = self.getWristArmRotation()
            currentElbowRotation = self.getElbowArmRotation()
            currentShoulderRotation = self.getShoulderArmRotation()
            wristAngle = (
                self.thetaProfiledPID.calculate(
                    angleModulus(
                        (
                            currentWristRotation
                            + currentElbowRotation
                            + currentShoulderRotation
                        ).radians()
                    ),
                    angleModulus(pose.rotation().radians()),
                )
                + currentWristRotation.radians()
            )

        self.targetElbow = Pose2d(targetTwoLink, pose.rotation())

        if desiredInterpolation == ArmSubsystem.InterpolationMethod.JointSpace:
            currentWristRotation = self.getWristArmRotation()
            currentElbowRotation = self.getElbowArmRotation()
            currentShoulderRotation = self.getShoulderArmRotation()
            startAngle = (
                self.shoulderPID.calculate(
                    currentShoulderRotation.radians(), startAngle
                )
                + currentShoulderRotation.radians()
            )
            endAngle = (
                self.elbowPID.calculate(currentElbowRotation.radians(), endAngle)
                + currentElbowRotation.radians()
            )
            wristAngle = (
                self.thetaProfiledPID.calculate(
                    currentWristRotation.radians(),
                    optimizeAngle(
                        currentWristRotation, Rotation2d(wristAngle)
                    ).radians(),
                )
                + currentWristRotation.radians()
            )

        self.setRelativeArmAngles(
            Rotation2d(startAngle),
            Rotation2d(endAngle),
            Rotation2d(wristAngle),
        )

    def setRelativeArmAngles(
        self, shoulder: Rotation2d, elbow: Rotation2d, wrist: Rotation2d
    ) -> None:
        currentWristRotation = self.getWristArmRotation()
        currentElbowRotation = self.getElbowArmRotation()
        currentShoulderRotation = self.getShoulderArmRotation()

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
            angleModulus(elbow.radians()),
            constants.kElbowMinAngle.radians(),
            constants.kElbowMaxAngle.radians(),
        )

        clampedWrist = clamp(
            optimizeAngle(currentWristRotation, wrist).radians(),
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
        trueElbowPos = clampedElbow + currentShoulderRotation.radians()
        trueWristPos = (
            clampedWrist
            + currentElbowRotation.radians()
            + currentShoulderRotation.radians()
        )

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
