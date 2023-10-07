from ctre.sensors import (
    AbsoluteSensorRange,
    SensorInitializationStrategy,
    CANCoder,
    CANCoderSimCollection,
)
from wpimath.geometry import Rotation2d

from util.ctrecheck import ctreCheckError

import constants


class CTREEncoder:
    def __init__(self, canId: int, offset: float, canbus: str = "") -> None:
        self.encoder = CANCoder(canId, canbus)
        self.offset = offset
        if not ctreCheckError(
            "configFactoryDefault",
            self.encoder.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        if not ctreCheckError(
            "configSensorInitializationStrategy",
            self.encoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "configMagnetOffset",
            self.encoder.configMagnetOffset(
                -1 * self.offset,  # invert the offset to zero the encoder
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "configAbsoluteSensorRange",
            self.encoder.configAbsoluteSensorRange(
                AbsoluteSensorRange.Signed_PlusMinus180,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "setPositionToAbsolute",
            self.encoder.setPositionToAbsolute(
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return

    def getPosition(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getAbsolutePosition())

    def getSimCollection(self) -> CANCoderSimCollection:
        return self.encoder.getSimCollection()
