import functools
import operator
from typing import List, Tuple
from commands2 import SubsystemBase
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, Timer
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

    def getCameraToTargetTransforms(self) -> List[Tuple[int, Transform3d]]:
        """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
        photonResult = self.camera.getLatestResult()
        if photonResult.hasTargets():
            return [
                (target.getFiducialId(), target.getBestCameraToTarget())
                for target in photonResult.getTargets()
                if target.getPoseAmbiguity() < constants.kPhotonvisionAmbiguityCutoff
            ]
        else:
            return []

    def periodic(self) -> None:
        self.estimatedPosition = self.drive.getPose()
        self.updateAdvantagescopePose()

        points = self.getCameraToTargetTransforms()

        if len(points) == 0:
            SmartDashboard.putNumberArray(constants.kFieldRelativeTargets, [])
            return

        derivedRobotPoses = [
            constants.kApriltagPositionDict[tag_id]
            + point.inverse()
            + constants.kLimelightRelativeToRobotTransform.inverse()
            for tag_id, point in points
        ]
        # construct a set of poses from each tag's relative position about where the robot would be

        for pose in derivedRobotPoses:
            # feed each value as a vision measurement into the kalman filter
            self.drive.estimator.addVisionMeasurement(
                pose.toPose2d(), Timer.getFPGATimestamp()
            )


        simApriltagPoses = [
            pose3dFrom2d(self.estimatedPosition)
            + constants.kLimelightRelativeToRobotTransform
            + point
            for _, point in points
        ]  # what the robot thinks the poses of every apriltag is

        sendablePoints = []

        for point in simApriltagPoses:
            x = point.X()
            y = point.Y()
            z = point.Z()
            rotationQuaternion = point.rotation().getQuaternion()
            w_rot = rotationQuaternion.W()
            x_rot = rotationQuaternion.X()
            y_rot = rotationQuaternion.Y()
            z_rot = rotationQuaternion.Z()

            sendablePoints.append(
                [x, y, z, w_rot, x_rot, y_rot, z_rot]
            )  # https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md#cones

        SmartDashboard.putNumberArray(
            constants.kFieldRelativeTargets,
            functools.reduce(
                operator.add, sendablePoints, []
            ),  # adds all the values found within targets (converts [[]] to [])
        )

    def updateAdvantagescopePose(self) -> None:
        limelightPose3d = (
            pose3dFrom2d(self.estimatedPosition)
            + constants.kLimelightRelativeToRobotTransform
        )
        limelightPose = advantagescopeconvert.convertToSendablePoses([limelightPose3d])

        SmartDashboard.putNumberArray(constants.kLimelightPoseKey, limelightPose)
