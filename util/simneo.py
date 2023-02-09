from rev import CANSparkMax, REVLibError, SparkMaxLimitSwitch
from enum import Enum, auto


def revCheckError(name: str, errorCode: REVLibError) -> bool:
    if errorCode is not None and errorCode != REVLibError.kOk:
        print("ERROR: {}: {}".format(name, errorCode))
        return False
    return True


class NEOBrushless:
    class ControlMode(Enum):
        Position = auto()
        Velocity = auto()
        Percent = auto()

    class NeutralMode(Enum):
        Brake = auto()
        Coast = auto()

    class LimitSwitch(Enum):
        Forwards = auto()
        Backwards = auto()

    def __init__(
        self,
        canID: int,
        pidSlot: int = 0,
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isInverted: bool = False,
    ):
        self.motor = CANSparkMax(canID, CANSparkMax.MotorType.kBrushless)
        self.controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()

        self.forwardSwitch = self.motor.getForwardLimitSwitch(
            SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self.reverseSwitch = self.motor.getReverseLimitSwitch(
            SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        if not revCheckError("factoryConfig", self.motor.restoreFactoryDefaults()):
            return
        if not revCheckError("setP", self.controller.setP(pGain, pidSlot)):
            return
        if not revCheckError("setI", self.controller.setI(iGain, pidSlot)):
            return
        if not revCheckError("setD", self.controller.setD(dGain, pidSlot)):
            return

        self.motor.setInverted(isInverted)

    def set(self, controlMode: ControlMode, demand: float):
        """input is in rotations or rpm"""
        if controlMode == NEOBrushless.ControlMode.Velocity:
            self.controller.setReference(demand, CANSparkMax.ControlType.kVelocity)
        elif controlMode == NEOBrushless.ControlMode.Position:
            self.controller.setReference(demand, CANSparkMax.ControlType.kPosition)
        elif controlMode == NEOBrushless.ControlMode.Percent:
            self.motor.set(demand)

    def get(self, controlMode: ControlMode) -> float:
        if controlMode == NEOBrushless.ControlMode.Velocity:
            return self.encoder.getVelocity()
        elif controlMode == NEOBrushless.ControlMode.Position:
            return self.encoder.getPosition()
        elif controlMode == NEOBrushless.ControlMode.Percent:
            return self.motor.get()

    def setNeutralOutput(self, output: NeutralMode) -> None:
        self.motor.setIdleMode(
            CANSparkMax.IdleMode.kBrake
            if output == NEOBrushless.NeutralMode.Brake
            else CANSparkMax.IdleMode.kCoast
        )

    def neutralOutput(self) -> None:
        self.motor.set(0)

    def getLimitSwitch(self, switch: LimitSwitch) -> bool:
        if switch == NEOBrushless.LimitSwitch.Forwards:
            return self.forwardSwitch.get()
        if switch == NEOBrushless.LimitSwitch.Backwards:
            return self.reverseSwitch.get()
        return False

    def setSmartCurrentLimit(self, limit: int = 15) -> None:
        self.motor.setSmartCurrentLimit(limit)
        """15 amps"""
