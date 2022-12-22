# this module is a placeholder currently as 2023 vision will include apriltags

import math
import functools
import operator
from typing import List, Tuple
from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.geometry import Transform3d, Pose3d, Pose2d, Rotation3d

import constants


class VisionSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

    def getCameraToTargetTransforms(self) -> List[Tuple[int, Transform3d]]:
        """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
        # TODO: actual implementation from PhotonVision or other source
        robotPose2d = Pose2d(
            *SmartDashboard.getNumberArray(
                constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
            )
        )
        robotPose3d = Pose3d(
            robotPose2d.X(),
            robotPose2d.Y(),
            0,
            Rotation3d(0, 0, robotPose2d.rotation().radians()),
        )

        limelightPosition3d = robotPose3d + constants.kLimelightRelativeToRobotTransform

        validPoints = [(0, limelightPosition3d)]

        for id, point in enumerate(constants.kApriltagPositionDict.values()):
            poseDelta = point - limelightPosition3d

            if (
                abs(math.atan2(poseDelta.Y(), poseDelta.X()))
                < constants.kLimelightMaxHorizontalFoV.radians()
                and abs(math.atan2(poseDelta.Z(), poseDelta.X()))
                < constants.kLimelightMaxVerticalFoV.radians()
            ):  # view frustum restrictions, inverse tan on XY and XZ axis to get horizontal and vertical restrictions
                validPoints.append((id, Pose3d().transformBy(poseDelta)))

        return validPoints

    def periodic(self) -> None:
        points = self.getCameraToTargetTransforms()


        transformedPoints = [
            (id, point + constants.kLimelightRelativeToRobotTransform)
            for id, point in points
        ]

        sendablePoints = []

        for id, point in transformedPoints:
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
