from typing import List
from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from pathplannerlib import PathPlannerTrajectory
from wpimath.geometry import Pose2d

from commands.auto.autohelper import trajectoryFromFile
from commands.auto.followtrajectory import FollowTrajectory
from commands.resetdrive import ResetDrive
from subsystems.drivesubsystem import DriveSubsystem


class AutonomousRoutine(SequentialCommandGroup):
    def __init__(
        self,
        drive: DriveSubsystem,
        name: str,
        simultaneousCommands: List[Command],
        **markerIden: Command
    ):
        self.markerMap = markerIden
        paths = trajectoryFromFile(name)
        followCommands = [
            SequentialCommandGroup(
                self.stopEventGroup(path.getStartStopEvent()),
                FollowTrajectory(drive, path, path.getMarkers(), markerIden),
                self.stopEventGroup(path.getEndStopEvent()),
            )
            for path in paths
        ]

        super().__init__(
            ResetDrive(
                drive,
                Pose2d(
                    paths[0].getInitialState().pose.translation(),
                    paths[0].getInitialState().holonomicRotation,
                ),
            ),
            ParallelCommandGroup(
                SequentialCommandGroup(*followCommands), *simultaneousCommands
            ),
        )

    def getStopEventCommands(
        self, stopEvent: PathPlannerTrajectory.StopEvent
    ) -> Command:
        commands = [
            self.markerMap.get(name)
            for name in stopEvent.names
            if name in self.markerMap.keys()
        ]
        if (
            stopEvent.executionBehavior
            == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL
        ):
            return ParallelCommandGroup(commands)
        elif (
            stopEvent.executionBehavior
            == PathPlannerTrajectory.StopEvent.ExecutionBehavior.SEQUENTIAL
        ):
            return SequentialCommandGroup(commands)
        else:
            raise NotImplementedError("That execution behaviour is not implemented")

    def stopEventGroup(self, stopEvent: PathPlannerTrajectory.StopEvent) -> Command:
        eventCommands = self.getStopEventCommands(stopEvent)
        if (
            stopEvent.waitBehavior
            == PathPlannerTrajectory.StopEvent.WaitBehavior.BEFORE
        ):
            return SequentialCommandGroup(
                WaitCommand(stopEvent.waitTime), eventCommands
            )
        elif (
            stopEvent.waitBehavior == PathPlannerTrajectory.StopEvent.WaitBehavior.AFTER
        ):
            return SequentialCommandGroup(
                eventCommands, WaitCommand(stopEvent.waitTime)
            )
        elif (
            stopEvent.waitBehavior
            == PathPlannerTrajectory.StopEvent.WaitBehavior.DEADLINE
        ):
            return ParallelDeadlineGroup(WaitCommand(stopEvent.waitTime), eventCommands)
        elif (
            stopEvent.waitBehavior
            == PathPlannerTrajectory.StopEvent.WaitBehavior.MINIMUM
        ):
            return ParallelCommandGroup(WaitCommand(stopEvent.waitTime), eventCommands)
        else:
            return eventCommands