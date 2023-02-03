import math
from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import (
    Color,
    Color8Bit,
    Mechanism2d,
    Preferences,
    SmartDashboard,
)
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
from util.convenientmath import clamp, pose3dFrom2d
from util.simcoder import CTREEncoder

from util.simfalcon import Falcon

import constants


class ArmSubsystem(SubsystemBase):
    class ArmState(Enum):
        Stored = auto()
        Mid = auto()
        HumanStation = auto()
        Top = auto()
        GroundLoading = auto()
        OverrideValue = auto()

        def position(self) -> Pose2d:
            if self == ArmSubsystem.ArmState.Stored:
                return constants.kArmStoredPosition
            elif self == ArmSubsystem.ArmState.Mid:
                return constants.kArmMidScorePosition
            elif self == ArmSubsystem.ArmState.HumanStation:
                return constants.kArmDoubleSubstationPosition
            elif self == ArmSubsystem.ArmState.Top:
                return constants.kArmTopScorePosition
            elif self == ArmSubsystem.ArmState.GroundLoading:
                return constants.kArmGroundIntakePosition
            return constants.kArmStoredPosition

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.mech = Mechanism2d(90, 90)
        self.state = ArmSubsystem.ArmState.Stored
        self.armFF = ArmFeedforward(0, constants.kShoulderArmFFFactor, 0, 0)

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
            useDINSim=False,
        )
        self.elbowArm.setNeutralMode(Falcon.NeutralMode.Break)
        self.elbowEncoder = CTREEncoder(
            constants.kElbowArmCANCoderID, constants.kElbowArmCANCoderOffset
        )

        self.shoulderArm = Falcon(
            constants.kShoulderArmCANId,
            constants.kArmPIDSlot,
            constants.kShoulderArmPGain,
            constants.kShoulderArmIGain,
            constants.kShoulderArmDGain,
            constants.kShoulderArmInverted,
            useDINSim=False,
        )
        self.shoulderArm.setNeutralMode(Falcon.NeutralMode.Break)
        self.shoulderEncoder = CTREEncoder(
            constants.kShoulderArmCANCoderID, constants.kShoulderArmCANCoderOffset
        )

        self.wristArm = Falcon(
            constants.kWristArmCANId,
            constants.kArmPIDSlot,
            constants.kWristArmPGain,
            constants.kWristArmIGain,
            constants.kWristArmDGain,
            constants.kWristArmInverted,
            useDINSim=False,
        )
        self.wristArm.setNeutralMode(Falcon.NeutralMode.Break)
        self.wristEncoder = CTREEncoder(
            constants.kWristArmCANCoderID, constants.kWristArmCANCoderOffset
        )

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
        self.targetPose = Pose2d()
        self.targetElbow = Pose2d()

        self.totalCOM = Translation2d()
        self.elbowRelativeCOM = Translation2d()
        self.wristRelativeCOM = Translation2d()

        self.reset()
        Preferences.initBoolean(constants.kArmSmoothKey, True)
        Preferences.initBoolean(constants.kArmObeyEndstopsKey, True)

    def reset(self) -> None:
        shoulderEncoderPosition = self.shoulderEncoder.getPosition().radians()
        elbowEncoderPosition = self.elbowEncoder.getPosition().radians()
        wristEncoderPosition = self.wristEncoder.getPosition().radians()

        self.shoulderArm.setEncoderPosition(
            shoulderEncoderPosition
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kShoulderArmGearRatio
        )
        self.elbowArm.setEncoderPosition(
            (elbowEncoderPosition + shoulderEncoderPosition)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kElbowArmGearRatio
        )
        self.wristArm.setEncoderPosition(
            (shoulderEncoderPosition + elbowEncoderPosition + wristEncoderPosition)
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristArmGearRatio
        )

    def getElbowArmRotation(self) -> Rotation2d:
        return (
            Rotation2d(
                self.elbowArm.get(Falcon.ControlMode.Position)
                / constants.kElbowArmGearRatio
                / constants.kTalonEncoderPulsesPerRadian
            )
            - self.getShoulderArmRotation()
        )  # 4 bar to relative

    def getShoulderArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.shoulderArm.get(Falcon.ControlMode.Position)
            / constants.kShoulderArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        )

    def getWristArmRotation(self) -> Rotation2d:
        return (
            Rotation2d(
                self.wristArm.get(Falcon.ControlMode.Position)
                / constants.kWristArmGearRatio
                / constants.kTalonEncoderPulsesPerRadian
            )
            - self.getShoulderArmRotation()
            - self.getElbowArmRotation()
        )  # 4 bar to relative

    def getElbowArmEncoderRotation(self) -> Rotation2d:
        return (
            self.elbowEncoder.getPosition()
        )  # encoder positions are already relative to previous joint

    def getShoulderArmEncoderRotation(self) -> Rotation2d:
        return self.shoulderEncoder.getPosition()

    def getWristArmEncoderRotation(self) -> Rotation2d:
        return self.wristEncoder.getPosition()

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
        SmartDashboard.putNumber(
            constants.kElbowArmRotationKey, self.getElbowArmRotation().degrees()
        )
        SmartDashboard.putNumber(
            constants.kShoulderArmRotationKey, self.getShoulderArmRotation().degrees()
        )
        SmartDashboard.putNumber(
            constants.kWristArmRotationKey, self.getWristArmRotation().degrees()
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

    def canElbowReachPosition(self, position: Translation2d):
        return (
            position.distance(Translation2d())
            < constants.kArmshoulderLength + constants.kArmelbowLength
            and position.distance(Translation2d())
            > constants.kArmshoulderLength - constants.kArmelbowLength
        )

    def nearestPossibleElbowPosition(self, position: Translation2d) -> Translation2d:
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

    def setEndEffectorPosition(self, pose: Pose2d):
        self.targetPose = pose

        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmwristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmwristLength * pose.rotation().sin(),
        )

        targetTwoLink = twoLinkPosition
        if Preferences.getBoolean(constants.kArmSmoothKey, True):
            currentWristPose = self.getWristPosition()
            targetTwoLink = Translation2d(
                self.xProfiledPID.calculate(currentWristPose.X(), twoLinkPosition.X())
                + currentWristPose.X(),
                self.yProfiledPID.calculate(currentWristPose.Y(), twoLinkPosition.Y())
                + currentWristPose.Y(),
            )
        while not self.canElbowReachPosition(targetTwoLink):
            targetTwoLink = self.nearestPossibleElbowPosition(targetTwoLink)

        endAngle = math.acos(
            (
                targetTwoLink.X() * targetTwoLink.X()
                + targetTwoLink.Y() * targetTwoLink.Y()
                - constants.kArmelbowLength * constants.kArmelbowLength
                - constants.kArmshoulderLength * constants.kArmshoulderLength
            )
            / (2 * constants.kArmelbowLength * constants.kArmshoulderLength)
        )

        startAngle = math.atan2(targetTwoLink.Y(), targetTwoLink.X()) - math.atan2(
            math.sin(endAngle) * constants.kArmelbowLength,
            constants.kArmshoulderLength
            + math.cos(endAngle) * constants.kArmelbowLength,
        )
        wristAngle = pose.rotation().radians() - startAngle - endAngle

        if Preferences.getBoolean(constants.kArmSmoothKey, True):
            currentWristRotation = self.getWristArmRotation()
            currentElbowRotation = self.getElbowArmRotation()
            currentShoulderRotation = self.getShoulderArmRotation()
            wristAngle = (
                self.thetaProfiledPID.calculate(
                    (
                        currentWristRotation
                        + currentElbowRotation
                        + currentShoulderRotation
                    ).radians(),
                    pose.rotation().radians(),
                )
                + currentWristRotation.radians()
            )

        self.targetElbow = Pose2d(targetTwoLink, pose.rotation())

        self.setRelativeArmAngles(
            Rotation2d(angleModulus(startAngle)),
            Rotation2d(angleModulus(endAngle)),
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
            shoulder.radians(),
            constants.kShoulderMinAngle.radians(),
            constants.kShoulderMaxAngle.radians(),
        )
        clampedElbow = clamp(
            elbow.radians(),
            constants.kElbowMinAngle.radians(),
            constants.kElbowMaxAngle.radians(),
        )

        clampedWrist = clamp(
            wrist.radians(),
            constants.kWristMinAngle.radians(),
            constants.kWristMaxAngle.radians(),
        )

        if not Preferences.getBoolean(constants.kArmObeyEndstopsKey, True):
            clampedShoulder = shoulder.radians()
            clampedElbow = elbow.radians()
            clampedWrist = wrist.radians()

        trueShoulderPos = clampedShoulder
        trueElbowPos = clampedElbow + trueShoulderPos
        trueWristPos = clampedWrist + trueElbowPos

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
