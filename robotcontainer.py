import os
import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button


import constants

from commands.resetdrive import ResetDrive
from commands.complexauto import ComplexAuto
from commands.drivedistance import DriveDistance
from commands.drivetotarget import DriveToTarget
from commands.drive.targetrelativedrive import TargetRelativeDrive
from commands.drive.robotrelativedrive import RobotRelativeDrive
from commands.drive.absoluterelativedrive import AbsoluteRelativeDrive
from commands.drive.drivewaypoint import DriveWaypoint
from commands.defensestate import DefenseState
from commands.auto.autonomousaction import AutonomousRoutine
from commands.grippercubecone import (
    GripperCubeGrab,
    GripperConeGrab,
    GripperHoldingState,
)

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.loggingsubsystem import LoggingSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.grippersubsystem import GripperSubsystem

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

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.SequentialCommandGroup(
            ResetDrive(self.drive),
            DriveDistance(
                4 * constants.kWheelCircumference,
                constants.kAutoDriveSpeedFactor,
                DriveDistance.Axis.X,
                self.drive,
            ),
        )
        self.grip.setDefaultCommand(GripperHoldingState(self.grip))

        # A complex auto routine that drives to the target, drives forward, waits, drives back
        self.complexAuto = ComplexAuto(self.drive)

        # A routine that drives to the target with a given offset
        self.driveToTarget = DriveToTarget(self.drive, constants.kAutoTargetOffset)

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            self.chooser.addOption(
                relevantName, AutonomousRoutine(self.drive, relevantName, [])
            )

        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            AbsoluteRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
            )
        )
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        commands2.button.JoystickButton(*self.operatorInterface.turboSpeed).whileHeld(
            AbsoluteRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kTurboSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kTurboSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
                self.operatorInterface.chassisControls.rotationY,
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
            *self.operatorInterface.targetRelativeCoordinateModeControl
        ).whileHeld(
            TargetRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.alignClosestWaypoint
        ).whileHeld(
            DriveWaypoint(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
            )
        )

        commands2.button.JoystickButton(*self.operatorInterface.resetGyro).whenPressed(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.defenseStateControl
        ).whileHeld(DefenseState(self.drive))

        commands2.button.JoystickButton(
            *self.operatorInterface.driveToTargetControl
        ).whenHeld(DriveToTarget(self.drive, constants.kAutoTargetOffset))

        # gripper subsystem related calls

        commands2.button.JoystickButton(*self.operatorInterface.gripCube).whileHeld(
            GripperCubeGrab(self.grip)
        )
        commands2.button.JoystickButton(*self.operatorInterface.gripCone).whileHeld(
            GripperConeGrab(self.grip)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
