from math import pi
import functools
import operator
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
    ) -> None:
        CommandBase.__init__(self)

        self.drive = drive
        self.markers = markers
        self.markerMap = markerMap

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

    def initialize(self):
        self.timer.reset()
        self.timer.start()

        self.unpassedMarkers = []
        self.unpassedMarkers.extend(self.markers)

        initialState = self.trajectory.getInitialState()
        allianceRespectiveStartingPoint = self.getAllianceRespectivePoint(
            initialState.pose, initialState.holonomicRotation
        )
        self.drive.resetOdometryAtPosition(
            Pose2d(
                allianceRespectiveStartingPoint[0].translation(),
                allianceRespectiveStartingPoint[1],
            ),
        )

        self.setControllers()

        transformedStates = [
            self.getAllianceRespectivePoint(state.pose, state.pose.rotation())
            for state in self.originTrajectory.getStates()
        ]

        SmartDashboard.putNumberArray(
            constants.kAutonomousPathKey,
            functools.reduce(
                operator.add,
                [
                    [state[0].X(), state[0].Y(), state[1].radians()]
                    for state in transformedStates
                ],
                [],
            ),
        )
        DataLogManager.log("begin trajectory")

    def getAllianceRespectivePoint(
        self, pose: Pose2d, holonomicRotation: Rotation2d
    ) -> Tuple[Pose2d, Rotation2d]:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return (
                Pose2d(
                    constants.kFieldLength - pose.X(),
                    pose.Y(),
                    -pose.rotation() + Rotation2d(pi),
                ),
                -holonomicRotation + Rotation2d(pi),
            )
        else:
            return (pose, holonomicRotation)

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
                currentState.X() - allianceRespectiveDesiredState[0].X(),
                currentState.Y() - allianceRespectiveDesiredState[0].Y(),
                (currentState.rotation() - allianceRespectiveDesiredState[1]).radians(),
            ],
        )
        SmartDashboard.putNumberArray(
            constants.kAutonomousPathSample,
            [
                allianceRespectiveDesiredState[0].X(),
                allianceRespectiveDesiredState[0].Y(),
                allianceRespectiveDesiredState[1].radians(),
            ],
        )

        targetChassisSpeeds = self.controller.calculate(
            currentState,
            allianceRespectiveDesiredState[0],
            desiredState.velocity,
            allianceRespectiveDesiredState[1],
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
        return Pose2d(
            finalAllianceRespectivePose[0].translation(),
            finalAllianceRespectivePose[1],
        )

    def end(self, _interrupted: bool) -> None:
        self.drive.resetOdometryAtPosition(self.getFinalPosition())
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
        DataLogManager.log("end trajectory")
