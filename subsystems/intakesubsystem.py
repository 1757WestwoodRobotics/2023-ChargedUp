from enum import Enum, auto
from commands2 import SubsystemBase
from util.simneo import NEOBrushless

import constants

class IntakeSubsystem(SubsystemBase):

    class State(Enum):
        Idle = auto()
        Intaking = auto()
        Outtaking = auto()


    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.motor = NEOBrushless(
            constants.kIntakeCANID, 
            constants.kIntakePIDSlot, 
            constants.kIntakePGain, 
            constants.kIntakeIGain, 
            constants.kIntakeDGain)

        self.motor.setNeutralOutput(NEOBrushless.NeutralMode.Brake)

        self.state = IntakeSubsystem.State.Idle
        
    def periodic(self) -> None:
        if self.state == IntakeSubsystem.State.Idle:
            self.motor.neutralOutput()        
        elif self.state == IntakeSubsystem.State.Intaking:
            self.motor.set(NEOBrushless.ControlMode.Percent, constants.kIntakeMotorPercent)
        elif self.state == IntakeSubsystem.State.Outtaking:
            self.motor.set(NEOBrushless.ControlMode.Percent, -constants.kIntakeMotorPercent)

    def setIntaking(self) -> None:
        self.state = IntakeSubsystem.State.Intaking

    def setOuttaking(self) -> None:
        self.state = IntakeSubsystem.State.Outtaking

    def setIdle(self) -> None:
        self.state = IntakeSubsystem.State.Idle
