import math
from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import (
    Color,
    Color8Bit,
    DriverStation,
    Mechanism2d,
    RobotBase,
    SmartDashboard,
)
from wpimath._controls._controls.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from util.angleoptimize import optimizeAngle
from util.simcoder import CTREEncoder

from util.simfalcon import Falcon

import constants


class ArmSubsystem(SubsystemBase):
    class ArmState(Enum):
        Stored = auto()
        Mid = auto()
        HumanStation = auto()
        Top = auto()
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

        armPivot = self.mech.getRoot("ArmPivot", 53.5, 8)
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
            useDINSim=False
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
            useDINSim=False
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
            useDINSim=False
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

        self.reset()

    def reset(self) -> None:
        self.shoulderArm.setEncoderPosition(
            self.shoulderEncoder.getPosition().radians()
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kShoulderArmGearRatio
        )
        self.elbowArm.setEncoderPosition(
            self.elbowEncoder.getPosition().radians()
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kElbowArmGearRatio
        )
        self.wristArm.setEncoderPosition(
            self.wristEncoder.getPosition().radians()
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristArmGearRatio
        )

    def getElbowArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.elbowArm.get(Falcon.ControlMode.Position)
            / constants.kElbowArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        )

    def getShoulderArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.shoulderArm.get(Falcon.ControlMode.Position)
            / constants.kShoulderArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        )

    def getWristArmRotation(self) -> Rotation2d:
        return Rotation2d(
            self.wristArm.get(Falcon.ControlMode.Position)
            / constants.kWristArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
        )

    def updateMechanism(self) -> None:
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
            self.setEndEffectorPosition(self.state.position())
        self.updateMechanism()

    def setEndEffectorPosition(self, pose: Pose2d):
        twoLinkGoalPosition = Translation2d(
            pose.X() - constants.kArmwristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmwristLength * pose.rotation().sin(),
        )
        twoLinkPosition = twoLinkGoalPosition

        endAngle = math.acos(
            twoLinkPosition.X() * twoLinkPosition.X()
            + twoLinkPosition.Y() * twoLinkPosition.Y()
            - constants.kArmelbowLength * constants.kArmelbowLength
            - constants.kArmshoulderLength
            * constants.kArmshoulderLength
            / (2 * constants.kArmelbowLength * constants.kArmshoulderLength)
        )

        startAngle = math.atan2(twoLinkPosition.Y(), twoLinkPosition.X()) - math.atan2(
            math.sin(endAngle) * constants.kArmelbowLength,
            constants.kArmshoulderLength + math.cos(endAngle) * constants.kArmelbowLength,
        )
        wristAngle = pose.rotation().radians() - startAngle - endAngle

        self.setRelativeArmAngles(
            Rotation2d(startAngle), Rotation2d(endAngle), Rotation2d(wristAngle)
        )

    def setRelativeArmAngles(
        self, shoulder: Rotation2d, elbow: Rotation2d, wrist: Rotation2d
    ) -> None:
        shoulderArmEncoderPulses = (
            shoulder.radians()
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kShoulderArmGearRatio
        )
        elbowArmEncoderPulses = (
            elbow.radians()
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kElbowArmGearRatio
        )
        wristArmEncoderPulses = (
            wrist.radians()
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristArmGearRatio
        )

        self.elbowArm.set(Falcon.ControlMode.Position, elbowArmEncoderPulses)
        self.shoulderArm.set(
            Falcon.ControlMode.Position,
            shoulderArmEncoderPulses,
            self.armFF.calculate(shoulder.radians(), 0, 0),
        )
        self.wristArm.set(Falcon.ControlMode.Position, wristArmEncoderPulses)
