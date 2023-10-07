from math import pi
from typing import Dict, List, Tuple
from commands2 import Command, CommandBase
from pathplannerlib import PathPlannerTrajectory
from wpilib import DataLogManager, DriverStation, SmartDashboard, Timer
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfileRadians

from subsystems.drivesubsystem import DriveSubsystem
import constants


class FollowTrajectory(CommandBase):
    unpassedMarkers: List[PathPlannerTrajectory.EventMarker]

    def __init__(
        self,
        drive: DriveSubsystem,
        trajectory: PathPlannerTrajectory,
        markers: List[PathPlannerTrajectory.EventMarker],
        markerMap: Dict[str, Command],
        doReset: bool = True,
    ) -> None:
        CommandBase.__init__(self)

        self.drive = drive
        self.markers = markers
        self.markerMap = markerMap

        self.doReset = doReset

        self.currentCommands: List[Tuple[Command, bool]] = []

        for marker in self.markers:
            for markerName in marker.names:
                if markerName in self.markerMap.keys():
                    reqs = self.markerMap[markerName].getRequirements()
                    self.addRequirements(
                        reqs
                    )  # make sure we require everything our sub commands require

        self.setControllers()

        self.originTrajectory = trajectory
        self.trajectory = trajectory

        self.timer = Timer()

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def setControllers(self) -> None:
        self.xController = PIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
        )
        self.yController = PIDController(
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
        )
        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.thetaController.enableContinuousInput(-pi, pi)

        self.controller = HolonomicDriveController(
            self.xController, self.yController, self.thetaController
        )

    @staticmethod
    def allianceRespectivePoseFromState(state):
        return FollowTrajectory.getAllianceRespectivePoint(
            state.pose, state.holonomicRotation
        )

    def initialize(self):
        self.timer.reset()
        self.timer.start()

        self.unpassedMarkers = []
        self.unpassedMarkers.extend(self.markers)

        initialState = self.trajectory.getInitialState()
        allianceRespectiveStartingPoint = self.getAllianceRespectivePoint(
            initialState.pose, initialState.holonomicRotation
        )
        if self.doReset:
            self.drive.setOdometryPosition(allianceRespectiveStartingPoint)

        self.setControllers()

        # transformedStates = [
        #     self.getAllianceRespectivePoint(state.pose, state.pose.rotation())
        #     for state in self.originTrajectory.getStates()
        # ]

        # SmartDashboard.putNumberArray(
        #     constants.kAutonomousPathKey,
        #     functools.reduce(
        #         operator.add,
        #         [
        #             [state[0].X(), state[0].Y(), state[1].radians()]
        #             for state in transformedStates
        #         ],
        #         [],
        #     ),
        # )
        DataLogManager.log("begin trajectory")

    @staticmethod
    def getAllianceRespectivePoint(
        pose: Pose2d, holonomicRotation: Rotation2d
    ) -> Pose2d:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return Pose2d(
                constants.kFieldLength - pose.X(),
                pose.Y(),
                -holonomicRotation + Rotation2d(pi),
            )
        else:
            return Pose2d(pose.translation(), holonomicRotation)

    def execute(self) -> None:
        for command, running in self.currentCommands:
            if not running:
                continue
            command.execute()

            if command.isFinished():
                command.end(False)
                running = False

        curTime = self.timer.get()
        currentState = self.drive.getPose()
        desiredState = self.trajectory.sample(curTime)

        allianceRespectiveDesiredState = self.getAllianceRespectivePoint(
            desiredState.pose, desiredState.holonomicRotation
        )

        SmartDashboard.putNumberArray(
            constants.kAutonomousPathError,
            [
                currentState.X() - allianceRespectiveDesiredState.X(),
                currentState.Y() - allianceRespectiveDesiredState.Y(),
                (
                    currentState.rotation() - allianceRespectiveDesiredState.rotation()
                ).radians(),
            ],
        )
        SmartDashboard.putNumberArray(
            constants.kAutonomousPathSample,
            [
                allianceRespectiveDesiredState.X(),
                allianceRespectiveDesiredState.Y(),
                allianceRespectiveDesiredState.rotation().radians(),
            ],
        )

        targetChassisSpeeds = self.controller.calculate(
            currentState,
            allianceRespectiveDesiredState,
            desiredState.velocity,
            allianceRespectiveDesiredState.rotation(),
        )

        SmartDashboard.putNumberArray(
            constants.kAutonomousChassisSpeeds,
            [targetChassisSpeeds.vx, targetChassisSpeeds.vy, targetChassisSpeeds.omega],
        )

        self.drive.arcadeDriveWithSpeeds(
            targetChassisSpeeds, DriveSubsystem.CoordinateMode.RobotRelative
        )

        # marker relatev stuff
        if len(self.unpassedMarkers) > 0 and curTime >= self.unpassedMarkers[0].time:
            marker = self.unpassedMarkers.pop(0)
            for name in marker.names:
                if name in self.markerMap:
                    eventCommand = self.markerMap[name]
                    eventCommand.initialize()
                    self.currentCommands.append((eventCommand, True))

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.trajectory.getTotalTime())

    def getFinalPosition(self) -> Pose2d:
        endState = self.trajectory.getEndState()
        finalAllianceRespectivePose = self.getAllianceRespectivePoint(
            endState.pose, endState.holonomicRotation
        )
        return finalAllianceRespectivePose

    def end(self, _interrupted: bool) -> None:
        if self.doReset:
            self.drive.resetOdometryAtPosition(self.getFinalPosition())

        for command, running in self.currentCommands:
            if running:
                command.end(True)
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
        DataLogManager.log("end trajectory")
