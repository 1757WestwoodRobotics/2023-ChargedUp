from typing import List, Tuple
from commands2 import SubsystemBase
from ntcore import NetworkTableInstance
from photonvision import PoseStrategy, RobotPoseEstimator
from robotpy_apriltag import AprilTagField
import robotpy_apriltag
from wpilib import SmartDashboard
from wpimath.geometry import (
    Transform3d,
)
from photonvision import PhotonCamera

import constants
from subsystems.drivesubsystem import DriveSubsystem
from util import advantagescopeconvert
from util.convenientmath import pose3dFrom2d


class VisionSubsystem(SubsystemBase):
    def __init__(self, drive: DriveSubsystem) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive

        self.camera = PhotonCamera(
            NetworkTableInstance.getDefault(), constants.kPhotonvisionCameraName
        )
        field = robotpy_apriltag.loadAprilTagLayoutField(AprilTagField.k2023ChargedUp)
        self.estimator = RobotPoseEstimator(
            field,
            PoseStrategy.LOWEST_AMBIGUITY,
            [(self.camera, constants.kLimelightRelativeToRobotTransform)],
        )

    def getCameraToTargetTransforms(
        self,
    ) -> Tuple[List[Tuple[int, Transform3d]], float]:
        """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
        photonResult = self.camera.getLatestResult()
        if photonResult.hasTargets():
            return (
                [
                    (target.getFiducialId(), target.getBestCameraToTarget())
                    for target in photonResult.getTargets()
                    if target.getPoseAmbiguity()
                    < constants.kPhotonvisionAmbiguityCutoff
                ],
                photonResult.getTimestamp(),
            )
        else:
            return ([], 0)

    def periodic(self) -> None:
        self.estimatedPosition = self.drive.getPose()
        self.updateAdvantagescopePose()

        botPose, _ = self.estimator.update()

        self.drive.visionEstimate = botPose.toPose2d()

        SmartDashboard.putBoolean(
            constants.kRobotVisionPoseArrayKeys.validKey, self.camera.hasTargets()
        )
        SmartDashboard.putNumberArray(
            constants.kRobotVisionPoseArrayKeys.valueKey,
            [
                self.drive.visionEstimate.X(),
                self.drive.visionEstimate.Y(),
                self.drive.visionEstimate.rotation().radians(),
            ],
        )

    def updateAdvantagescopePose(self) -> None:
        limelightPose3d = (
            pose3dFrom2d(self.estimatedPosition)
            + constants.kLimelightRelativeToRobotTransform
        )
        limelightPose = advantagescopeconvert.convertToSendablePoses([limelightPose3d])

        SmartDashboard.putNumberArray(constants.kLimelightPoseKey, limelightPose)
