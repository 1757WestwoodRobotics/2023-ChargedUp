from functools import reduce
from operator import add

from commands2 import SubsystemBase
from ntcore import NetworkTableInstance
from wpilib import PowerDistribution, SmartDashboard, DriverStation

from operatorinterface import OperatorInterface

import constants


class LoggingSubsystem(SubsystemBase):
    def __init__(self, oi: OperatorInterface) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        self.oi = oi
        self.dsTable = NetworkTableInstance.getDefault().getTable(
            constants.kJoystickKeyLogPrefix
        )

    def periodic(self) -> None:
        SmartDashboard.putData(self.pdh)

        for controller in self.oi.controllers.values():
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonCount",
                controller.getButtonCount(),
            )
            encodedButtonValue = reduce(
                add,
                [
                    controller.getRawButton(i) << i - 1
                    for i in range(controller.getButtonCount() + 1, 0, -1)
                ],
                0,
            )
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonValues", encodedButtonValue
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/AxisValues",
                [controller.getRawAxis(i) for i in range(controller.getAxisCount())],
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/POVs",
                [controller.getPOV(i) for i in range(controller.getPOVCount())],
            )

            self.dsTable.putBoolean("enabled", DriverStation.isEnabled())
            self.dsTable.putBoolean("auto", DriverStation.isAutonomous())
            self.dsTable.putString("alliance", str(DriverStation.getAlliance()))
            self.dsTable.putNumber("location", DriverStation.getLocation())
