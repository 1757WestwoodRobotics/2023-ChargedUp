from typing import Dict, List
from commands2 import (
    Command,
    CommandBase,
    FunctionalCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from pathplannerlib import PathPlannerTrajectory
from wpilib import DataLogManager

from commands.auto.autohelper import trajectoryFromFile
from commands.auto.followtrajectory import FollowTrajectory
from commands.resetdrive import ResetDrive
from subsystems.drivesubsystem import DriveSubsystem


class AutonomousRoutine(SequentialCommandGroup):
    markerMap: Dict[str, Command]

    def __init__(
        self,
        drive: DriveSubsystem,
        name: str,
        simultaneousCommands: List[Command],
    ):
        self.name = name
        self.markerMap = {  # later todo: actual implementation
            "store": WaitCommand(2),
            "top": WaitCommand(2),
            "mid": WaitCommand(2),
            "hybrid": WaitCommand(2),
            "engage": WaitCommand(5),
            "intake": WaitCommand(0.25),
            "outtake": WaitCommand(0.25),
        }
        self.paths = trajectoryFromFile(name)
        followCommands = [
            SequentialCommandGroup(
                self.stopEventGroup(path.getStartStopEvent()),
                FollowTrajectory(drive, path, path.getMarkers(), self.markerMap),
                self.stopEventGroup(path.getEndStopEvent()),
            )
            for path in self.paths
        ]

        super().__init__(
            ResetDrive(drive),
            ParallelCommandGroup(
                SequentialCommandGroup(*followCommands), *simultaneousCommands
            ),
        )
        self.setName(name)

    def execute(self) -> None:
        DataLogManager.log(f"Starting auto: {self.name}")
        return super().execute()

    def wrappedEventCommand(self, eventCommand: Command) -> CommandBase:
        return FunctionalCommand(
            eventCommand.initialize,
            eventCommand.execute,
            eventCommand.end,
            eventCommand.isFinished,
            list(eventCommand.getRequirements()),
        )

    def getStopEventCommands(
        self, stopEvent: PathPlannerTrajectory.StopEvent
    ) -> Command:
        commands = [
            self.wrappedEventCommand(self.markerMap.get(name))
            for name in stopEvent.names
            if name in self.markerMap
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
