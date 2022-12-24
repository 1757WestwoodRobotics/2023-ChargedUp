import math
import functools
import operator
from typing import List, Tuple
from commands2 import SubsystemBase
from wpilib import SmartDashboard, Timer
from wpimath.geometry import (
    Pose2d,
    Transform3d,
)

import constants
from subsystems.drivesubsystem import DriveSubsystem
from util.convenientmath import pose3dFrom2d


class VisionSubsystem(SubsystemBase):
    def __init__(self, drive: DriveSubsystem) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive

    def getCameraToTargetTransforms(self) -> List[Tuple[int, Transform3d]]:
        """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
        # TODO: actual implementation from PhotonVision or other source
        robotPose2d = Pose2d(
            *SmartDashboard.getNumberArray(constants.kSimRobotPoseArrayKey, [0, 0, 0])
        )
        robotPose3d = pose3dFrom2d(robotPose2d)

        limelightPosition3d = robotPose3d + constants.kLimelightRelativeToRobotTransform

        validPoints = []

        for tag_id, point in zip(
            constants.kApriltagPositionDict.keys(),
            constants.kApriltagPositionDict.values(),
        ):
            poseDelta = Transform3d(limelightPosition3d, point)

            if (
                abs(math.atan2(poseDelta.Y(), poseDelta.X()))
                < constants.kLimelightMaxHorizontalFoV.radians()
                and abs(math.atan2(poseDelta.Z(), poseDelta.X()))
                < constants.kLimelightMaxVerticalFoV.radians()
            ):  # view frustum restrictions, inverse tan on XY and XZ axis to get horizontal and vertical restrictions
                validPoints.append((tag_id, poseDelta))

        return validPoints

    def periodic(self) -> None:
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

        estimatedPosition = self.drive.getPose()

        simApriltagPoses = [
            pose3dFrom2d(estimatedPosition)
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
