import os
import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button

import constants

from commands.resetdrive import ResetDrive
from commands.complexauto import ComplexAuto
from commands.drivedistance import DriveDistance
from commands.drive.robotrelativedrive import RobotRelativeDrive
from commands.drive.fieldrelativedrive import FieldRelativeDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.defensestate import DefenseState
from commands.arm.demostate import DemoArm
from commands.arm.resetarm import ResetArm
from commands.arm.statearmposition import (
    DecreaseArmFudge,
    IncreaseArmFudge,
    SetArmPositionDoubleSubstation,
    SetArmPositionGroundCone,
    SetArmPositionGroundIntake,
    SetArmPositionMid,
    SetArmPositionOverride,
    SetArmPositionSafeTop,
    SetArmPositionSingleSubstation,
    SetArmPositionStored,
    SetArmPositionTop,
)
from commands.auto.autonomousaction import AutonomousRoutine
from commands.gripper import (
    GripperIntake,
    GripperOuttake,
    GripperHoldingState,
)
from commands.light.cubeLights import ConeLights, CubeLights
from commands.drive.chargestationautobalance import AutoBalance

from subsystems.armsubsystem import ArmSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.loggingsubsystem import LoggingSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.grippersubsystem import GripperSubsystem
from subsystems.lightsubsystem import LightSubsystem

from operatorinterface import OperatorInterface


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The operator interface (driver controls)
        self.operatorInterface = OperatorInterface()

        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.vision = VisionSubsystem(self.drive)
        self.log = LoggingSubsystem(self.operatorInterface)
        self.grip = GripperSubsystem()
        self.arm = ArmSubsystem()
        self.light = LightSubsystem()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.SequentialCommandGroup(
            ResetDrive(self.drive),
            DriveDistance(
                -4 * constants.kWheelCircumference,
                0.2,
                DriveDistance.Axis.X,
                self.drive,
            ),
        )
        self.grip.setDefaultCommand(GripperHoldingState(self.grip))

        # A complex auto routine that drives to the target, drives forward, waits, drives back
        self.complexAuto = ComplexAuto(self.drive)

        # A routine that drives to the target with a given offset

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            self.chooser.addOption(
                relevantName,
                AutonomousRoutine(self.drive, self.arm, self.grip, relevantName, []),
            )

        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            FieldRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kTurboSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kTurboSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
            )
        )
        self.arm.setDefaultCommand(SetArmPositionStored(self.arm))
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        commands2.button.POVButton(*self.operatorInterface.armMid).whileHeld(
            SetArmPositionMid(self.arm)
        )
        commands2.button.POVButton(*self.operatorInterface.armTop).whileHeld(
            SetArmPositionTop(self.arm)
        ).whenReleased(
            commands2.ParallelCommandGroup(
                SetArmPositionSafeTop(self.arm), commands2.WaitCommand(0.4)
            )
        )
        commands2.button.JoystickButton(
            *self.operatorInterface.armDoubleSubstation
        ).whileHeld(SetArmPositionDoubleSubstation(self.arm))
        commands2.button.JoystickButton(
            *self.operatorInterface.armSingleSubstation
        ).whileHeld(SetArmPositionSingleSubstation(self.arm))

        commands2.button.JoystickButton(*self.operatorInterface.armOverride).whileHeld(
            SetArmPositionOverride(self.arm)
        )
        commands2.button.POVButton(*self.operatorInterface.armGroundIntake).whileHeld(
            SetArmPositionGroundIntake(self.arm)
        )
        # .whenReleased(
        # commands2.ParallelCommandGroup(
        #     SetArmPositionSafeGround(self.arm), commands2.WaitCommand(0.7)
        # )
        # )
        commands2.button.POVButton(*self.operatorInterface.armGroundCone).whileHeld(
            SetArmPositionGroundCone(self.arm)
        ).whenReleased(
            commands2.ParallelCommandGroup(
                SetArmPositionMid(self.arm), commands2.WaitCommand(0.2)
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.armDemo
        ).toggleWhenPressed(DemoArm(self.arm))

        commands2.button.JoystickButton(*self.operatorInterface.resetArm).whenPressed(
            ResetArm(self.arm)
        )

        commands2.button.POVButton(
            *self.operatorInterface.armFudgeIncrease
        ).whenPressed(IncreaseArmFudge(self.arm))
        commands2.button.POVButton(
            *self.operatorInterface.armFudgeDecrease
        ).whenPressed(DecreaseArmFudge(self.arm))

        commands2.button.JoystickButton(*self.operatorInterface.turboSpeed).whileHeld(
            FieldRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.fieldRelativeCoordinateModeControl
        ).toggleWhenPressed(
            RobotRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.alignClosestWaypoint
        ).whileHeld(
            AngleAlignDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
            )
        )

        commands2.button.JoystickButton(*self.operatorInterface.resetGyro).whenPressed(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.defenseStateControl
        ).whileHeld(DefenseState(self.drive))

        # gripper subsystem related calls

        commands2.button.JoystickButton(*self.operatorInterface.gripIntake).whileHeld(
            GripperIntake(self.grip)
        )
        commands2.button.JoystickButton(*self.operatorInterface.gripOuttake).whileHeld(
            GripperOuttake(self.grip)
        )
        commands2.button.POVButton(*self.operatorInterface.lightCone).whenPressed(
            ConeLights(self.light)
        )

        commands2.button.POVButton(*self.operatorInterface.lightCube).whenPressed(
            CubeLights(self.light)
        )

        commands2.button.JoystickButton(*self.operatorInterface.AutoBalance).whileHeld(
            AutoBalance(self.drive)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
