from math import pi
from typing import Dict, List
from commands2 import Command, CommandBase, ParallelCommandGroup, ParallelDeadlineGroup, SequentialCommandGroup, WaitCommand
from pathplannerlib import PathPlannerTrajectory
from wpilib import DataLogManager, Timer
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
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

        for marker in self.markers:
            for markerName in marker.names:
                if markerName in self.markerMap.keys():
                    reqs = self.markerMap[markerName].getRequirements()
                    self.addRequirements(
                        reqs
                    )  # make sure we require everything our sub commands require

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

        self.trajectory = trajectory

        self.timer = Timer()

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)

    def initialize(self):
        self.timer.reset()
        self.timer.start()

        self.unpassedMarkers = []
        self.unpassedMarkers.extend(self.markers)

    def execute(self) -> None:
        curTime = self.timer.get()
        desiredState = self.trajectory.sample(curTime)
        targetChassisSpeeds = self.controller.calculate(
            self.drive.getPose(),
            desiredState.pose,
            desiredState.velocity,
            desiredState.holonomicRotation,
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
                    eventCommand.schedule()

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.trajectory.getTotalTime())

    def end(self, _interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
