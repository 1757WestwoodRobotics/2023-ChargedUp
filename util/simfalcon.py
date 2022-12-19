from enum import Enum, auto
from ctre import (
    ControlMode,
    ErrorCode,
    LimitSwitchNormal,
    LimitSwitchSource,
    NeutralMode,
    WPI_TalonFX,
)
from wpilib import RobotBase, SmartDashboard

from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor
from util.convenientmath import clamp
from util.ctrecheck import ctreCheckError

import constants


def createMotor(
    canID: int,
    canbus: str = "",
):
    if RobotBase.isReal():
        return WPI_TalonFX(canID, canbus)
    else:
        return SimFalcon(canID)


class SimFalcon:  # a simulated Falcon 500
    def __init__(
        self,
        canID: int,
    ) -> None:
        self.motor = WPI_TalonFX(canID)

        self.pidController = PIDController(0, 0, 0)
        self.motor.setSelectedSensorPosition(0)

        SmartDashboard.putNumber(
            f"{constants.kMotorBaseKey}/{self.motor.getDeviceID()}/encoder/value", 0
        )
        SmartDashboard.putNumber(
            f"{constants.kMotorBaseKey}/{self.motor.getDeviceID()}/output/value", 0
        )

        SmartDashboard.putBoolean(
            f"{constants.kMotorBaseKey}/{self.motor.getDeviceID()}/encoder/overwritten",
            False,
        )
        SmartDashboard.putBoolean(
            f"{constants.kMotorBaseKey}/{self.motor.getDeviceID()}/output/overwritten",
            False,
        )

    def configFactoryDefault(self, timeoutMs: int = 50) -> ErrorCode:
        return self.motor.configFactoryDefault(timeoutMs)

    def config_kP(self, slotIdx: int, value: float, timeoutMs: int = 0) -> ErrorCode:
        self.pidController.setP(value)
        return self.motor.config_kP(slotIdx, value, timeoutMs)

    def config_kI(self, slotIdx: int, value: float, timeoutMs: int = 0) -> ErrorCode:
        self.pidController.setI(value)
        return self.motor.config_kP(slotIdx, value, timeoutMs)

    def config_kD(self, slotIdx: int, value: float, timeoutMs: int = 0) -> ErrorCode:
        self.pidController.setD(value)
        return self.motor.config_kP(slotIdx, value, timeoutMs)

    def configForwardSoftLimitThreshold(
        self, reverseSensorLimit: float, timeoutMs: int = 0
    ) -> ErrorCode:
        return self.motor.configForwardSoftLimitThreshold(reverseSensorLimit, timeoutMs)

    def configReverseSoftLimitThreshold(
        self, reverseSensorLimit: float, timeoutMs: int = 0
    ) -> ErrorCode:
        return self.motor.configReverseSoftLimitThreshold(reverseSensorLimit, timeoutMs)

    def configForwardSoftLimitEnable(
        self, enable: bool, timeoutMs: int = 0
    ) -> ErrorCode:
        return self.motor.configForwardSoftLimitEnable(enable, timeoutMs)

    def configReverseSoftLimitEnable(
        self, enable: bool, timeoutMs: int = 0
    ) -> ErrorCode:
        return self.motor.configReverseSoftLimitEnable(enable, timeoutMs)

    def configForwardLimitSwitchSource(
        self,
        limitSwitchSource: LimitSwitchSource,
        normalOpenOrClose: LimitSwitchNormal,
        timeoutMs: int = 0,
    ) -> ErrorCode:
        return self.motor.configForwardLimitSwitchSource(
            limitSwitchSource, normalOpenOrClose, timeoutMs
        )

    def configReverseLimitSwitchSource(
        self,
        limitSwitchSource: LimitSwitchSource,
        normalOpenOrClose: LimitSwitchNormal,
        timeoutMs: int = 0,
    ) -> ErrorCode:
        return self.motor.configReverseLimitSwitchSource(
            limitSwitchSource, normalOpenOrClose, timeoutMs
        )

    def isRevLimitSwitchClosed(self):
        return self.motor.isRevLimitSwitchClosed()

    def isFwdLimitSwitchClosed(self):
        return self.motor.isFwdLimitSwitchClosed()

    def setInverted(self, invert: bool) -> None:
        self.motor.setInverted(invert)

    def getSelectedSensorPosition(self, pidIdx: int = 0) -> float:
        return self.motor.getSelectedSensorPosition(pidIdx)

    def setNeutralMode(self, mode: NeutralMode) -> None:
        self.motor.setNeutralMode(mode)

    def setSelectedSensorPosition(
        self, sensorPos: float, pidIdx: int = 0, timeoutMs: int = 50
    ) -> ErrorCode:
        return self.motor.setSelectedSensorPosition(sensorPos, pidIdx, timeoutMs)

    def getSelectedSensorVelocity(self, pidIdx: int = 0) -> float:
        return self.motor.getSelectedSensorVelocity(pidIdx)

    def neutralOutput(self):
        self.motor.neutralOutput()
        self.set(
            ControlMode.Velocity, 0
        )  # neutral mode is supposed to coast/brake the motor instead of driving to 0 velocity but for sim this works for now

    def get(self) -> float:
        return self.motor.get()

    def set(self, mode: ControlMode, demand: float) -> None:
        self.motor.set(mode, demand)
        currentPosition = self.motor.getSelectedSensorPosition()
        rawPercentOutput = 0
        if mode == ControlMode.Velocity:
            demandVelocity = demand / constants.kTalonVelocityPerAngularVelocity
            rawPercentOutput = demandVelocity / DCMotor.falcon500().freeSpeed
        elif mode == ControlMode.Position:
            positionError = self.pidController.calculate(currentPosition, demand)
            rawPercentOutput = (
                positionError / constants.kTalonEncoderPulsesPerRevolution
            )  # convert the change in encoder ticks into change into motor %

        clampedPercentOutput = clamp(rawPercentOutput, -1, 1)
        self.motor.setSelectedSensorPosition(
            currentPosition
            + (
                clampedPercentOutput
                # * 2 * tau # radians per second
                * DCMotor.falcon500().freeSpeed  # radians per second
                * constants.kTalonEncoderPulsesPerRadian  # encoder ticks per radian
                * constants.kRobotUpdatePeriod
            )
        )


class Falcon:
    class ControlMode(Enum):
        Position = auto()
        Velocity = auto()
        Percent  = auto()

    def __init__(
        self,
        canID: int,
        pidSlot: int = 0,
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isReversed: bool = False,
        canbus: str = "",
    ) -> None:
        self.motor = createMotor(canID, canbus)

        if not ctreCheckError(
            "configFactoryDefault", self.motor.configFactoryDefault()
        ):
            return
        if not ctreCheckError("config_kP", self.motor.config_kP(pidSlot, pGain)):
            return
        if not ctreCheckError("config_kI", self.motor.config_kI(pidSlot, iGain)):
            return
        if not ctreCheckError("config_kD", self.motor.config_kD(pidSlot, dGain)):
            return
        self.motor.setInverted(isReversed)

    def set(self, controlMode: ControlMode, demand: float) -> None:
        if controlMode ==  Falcon.ControlMode.Position:
            self.motor.set(ControlMode.Position, demand)
        elif controlMode == Falcon.ControlMode.Velocity:
            self.motor.set(ControlMode.Velocity, demand)
        elif controlMode == Falcon.ControlMode.Percent:
            self.motor.set(ControlMode.PercentOutput, demand)

    def get(self, controlMode: ControlMode) -> float:
        if controlMode ==  Falcon.ControlMode.Position:
            return self.motor.getSelectedSensorPosition()
        elif controlMode == Falcon.ControlMode.Velocity:
            return self.motor.getSelectedSensorVelocity()
        elif controlMode == Falcon.ControlMode.Percent:
            return self.motor.get()
