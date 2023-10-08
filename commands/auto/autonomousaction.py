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
from commands.arm.statearmposition import (
    SetArmHookState,
    SetArmPositionGroundIntake,
    SetArmPositionGroundYoshi,
    SetArmPositionMid,
    SetArmPositionSafeTop,
    SetArmPositionSingleSubstation,
    SetArmPositionStored,
    SetArmPositionTop,
)
from commands.defensestate import DefenseState
from commands.auto.autohelper import trajectoryFromFile
from commands.auto.followtrajectory import FollowTrajectory, GoToPoint
from commands.drive.chargestationautobalance import AutoBalance
from commands.gripper import GripperHoldingState, GripperIntake, GripperOuttake
from commands.light.cubeLights import ConeFlangeLights, CubeLights
from commands.resetdrive import ResetDrive
from subsystems.armsubsystem import ArmSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.grippersubsystem import GripperSubsystem
from subsystems.lightsubsystem import LightSubsystem


class AutonomousRoutine(SequentialCommandGroup):
    markerMap: Dict[str, Command]

    def __init__(
        self,
        drive: DriveSubsystem,
        arm: ArmSubsystem,
        grip: GripperSubsystem,
        light: LightSubsystem,
        name: str,
        simultaneousCommands: List[Command],
    ):
        self.name = name
        self.markerMap = {  # later todo: actual implementation
            "store": ParallelCommandGroup(
                SetArmPositionStored(arm),
                GripperHoldingState(grip),
                SetArmHookState(False),
            ),
            "top": SequentialCommandGroup(SetArmPositionTop(arm)),
            "mid": SequentialCommandGroup(SetArmPositionMid(arm), WaitCommand(1.2)),
            "midFlange": SequentialCommandGroup(
                ConeFlangeLights(light),
                SetArmPositionMid(arm),
                WaitCommand(0.2),
                SetArmHookState(True),
                WaitCommand(0.3),
            ),
            "safe": ParallelDeadlineGroup(
                WaitCommand(0.1), [SetArmPositionSafeTop(arm)]
            ),
            "cube": CubeLights(light),
            "safestore": ParallelDeadlineGroup(
                WaitCommand(0.4),
                [
                    SequentialCommandGroup(
                        ParallelCommandGroup(SetArmPositionSafeTop(arm)),
                        ParallelCommandGroup(
                            SetArmPositionStored(arm),
                            WaitCommand(0.2),
                            GripperHoldingState(grip),
                        ),
                    )
                ],
            ),
            "hybrid": SetArmPositionSingleSubstation(arm),
            "engage": AutoBalance(drive),
            "intake": ParallelCommandGroup(
                WaitCommand(0.25), GripperIntake(grip), SetArmPositionGroundIntake(arm)
            ),
            "intakeYoshi": ParallelCommandGroup(
                GripperIntake(grip), SetArmPositionGroundYoshi(arm)
            ),
            "outtake": SequentialCommandGroup(
                ParallelCommandGroup(GripperOuttake(grip), WaitCommand(0.3)),
            ),
        }
        self.paths = trajectoryFromFile(name)
        followCommands = [
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    self.stopEventGroup(path.getStartStopEvent()),
                    [
                        GoToPoint(
                            drive,
                            FollowTrajectory.allianceRespectivePoseFromState(
                                path.getInitialState()
                            ),
                        )
                    ],
                )
                if num == 0
                else WaitCommand(0),
                FollowTrajectory(drive, path, path.getMarkers(), self.markerMap, False),
                ParallelDeadlineGroup(
                    self.stopEventGroup(path.getEndStopEvent()),
                    [
                        GoToPoint(
                            drive,
                            FollowTrajectory.allianceRespectivePoseFromState(
                                path.getEndState()
                            ),
                        )
                        if drive
                        not in self.stopEventGroup(
                            path.getEndStopEvent()
                        ).getRequirements()
                        else WaitCommand(0)
                    ],
                ),
            )
            for num, path in enumerate(self.paths)
        ]

        super().__init__(
            ResetDrive(
                drive,
                FollowTrajectory.allianceRespectivePoseFromState(
                    self.paths[0].getInitialState()
                ),
            ),
            ParallelCommandGroup(
                SequentialCommandGroup(*followCommands), *simultaneousCommands
            ),
            DefenseState(drive),
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
