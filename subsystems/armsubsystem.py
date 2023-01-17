import math
from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import Color, Color8Bit, Mechanism2d, SmartDashboard
from wpimath.geometry import Pose2d, Translation2d

from util.simfalcon import Falcon

import constants


class ArmSubsystem(SubsystemBase):
    class ArmState(Enum):
        Stored = auto()
        Mid = auto()
        HumanStation = auto()
        Top = auto()

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

        self.armBottom = armPivot.appendLigament(
            "Arm Bottom",
            constants.kArmbottomLength / constants.kMetersPerInch,
            -90,
            10,
            Color8Bit(Color.kGold),
        )
        self.armTop = self.armBottom.appendLigament(
            "Arm Top",
            constants.kArmtopLength / constants.kMetersPerInch,
            45,
            10,
            Color8Bit(Color.kPurple),
        )
        self.armWrist = self.armTop.appendLigament(
            "Wrist",
            constants.kArmwristLength / constants.kMetersPerInch,
            10,
            10,
            Color8Bit(Color.kWhite),
        )

        SmartDashboard.putData("Arm Sim", self.mech)

        self.topArm = Falcon(
            constants.kTopArmCANId,
            constants.kArmPIDSlot,
            constants.kTopArmPGain,
            constants.kTopArmIGain,
            constants.kTopArmDGain,
            constants.kTopArmInverted,
        )
        self.bottomArm = Falcon(
            constants.kBottomArmCANId,
            constants.kArmPIDSlot,
            constants.kBottomArmPGain,
            constants.kBottomArmIGain,
            constants.kBottomArmDGain,
            constants.kBottomArmInverted,
        )
        self.wristArm = Falcon(
            constants.kWristPivotArmCANId,
            constants.kArmPIDSlot,
            constants.kWristPivotArmPGain,
            constants.kWristPivotArmIGain,
            constants.kWristPivotArmDGain,
            constants.kWristPivotArmInverted,
        )

    def updateMechanism(self) -> None:
        self.armTop.setAngle(
            self.topArm.get(Falcon.ControlMode.Position)
            / constants.kTopArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
            / constants.kRadiansPerDegree
        )
        self.armBottom.setAngle(
            self.bottomArm.get(Falcon.ControlMode.Position)
            / constants.kBottomArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
            / constants.kRadiansPerDegree
        )
        self.armWrist.setAngle(
            self.wristArm.get(Falcon.ControlMode.Position)
            / constants.kWristPivotArmGearRatio
            / constants.kTalonEncoderPulsesPerRadian
            / constants.kRadiansPerDegree
        )

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kArmStateKey, str(self.state))
        self.setEndEffectorPosition(self.state.position())
        self.updateMechanism()

    def setEndEffectorPosition(self, pose: Pose2d):

        twoLinkPosition = Translation2d(
            pose.X() - constants.kArmwristLength * pose.rotation().cos(),
            pose.Y() - constants.kArmwristLength * pose.rotation().sin(),
        )

        endAngle = math.acos(
            twoLinkPosition.X() * twoLinkPosition.X()
            + twoLinkPosition.Y() * twoLinkPosition.Y()
            - constants.kArmtopLength * constants.kArmtopLength
            - constants.kArmbottomLength
            * constants.kArmbottomLength
            / (2 * constants.kArmtopLength * constants.kArmbottomLength)
        )

        startAngle = math.atan2(twoLinkPosition.Y(), twoLinkPosition.X()) - math.atan2(
            math.sin(endAngle) * constants.kArmtopLength,
            constants.kArmbottomLength + math.cos(endAngle) * constants.kArmtopLength,
        )
        wristAngle = pose.rotation().radians() - startAngle - endAngle

        bottomArmEncoderPulses = (
            startAngle
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kBottomArmGearRatio
        )
        topArmEncoderPulses = (
            endAngle
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kTopArmGearRatio
        )
        wristArmEncoderPulses = (
            wristAngle
            * constants.kTalonEncoderPulsesPerRadian
            * constants.kWristPivotArmGearRatio
        )

        self.topArm.set(Falcon.ControlMode.Position, topArmEncoderPulses)
        self.bottomArm.set(Falcon.ControlMode.Position, bottomArmEncoderPulses)
        self.wristArm.set(Falcon.ControlMode.Position, wristArmEncoderPulses)
