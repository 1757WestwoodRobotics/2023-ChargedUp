#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import functools
import operator
import typing
import typing
import math
from photonvision import SimVisionSystem, SimVisionTarget
from ctre import CANCoderSimCollection, TalonFXSimCollection
from wpilib import RobotController, SmartDashboard
from wpilib.simulation import EncoderSim, PWMSim, SimDeviceSim, SingleJointedArmSim
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.system.plant import DCMotor
import wpimath.kinematics
from pyfrc.physics.core import PhysicsInterface

import constants


class SwerveModuleSim:
    def __init__(
        self,
        position: Translation2d,
        wheelMotorSim: PWMSim,
        wheelMotorType: DCMotor,
        driveMotorGearing,
        swerveMotorSim: PWMSim,
        swerveMotorType: DCMotor,
        steerMotorGearing,
        wheelEncoderSim: EncoderSim,
        swerveEncoderSim: EncoderSim,
    ) -> None:
        self.position = position
        self.wheelMotorSim = wheelMotorSim
        self.wheelMotorType = wheelMotorType
        self.driveMotorGearing = driveMotorGearing
        self.swerveMotorSim = swerveMotorSim
        self.swerveMotorType = swerveMotorType
        self.steerMotorGearing = steerMotorGearing
        self.wheelEncoderSim = wheelEncoderSim
        self.swerveEncoderSim = swerveEncoderSim

    def __str__(self) -> str:
        return f"pos: x={self.position.X():.2f} y={self.position.Y():.2f}"


class SwerveDriveSim:
    def __init__(self, swerveModuleSims: typing.Tuple[SwerveModuleSim, ...]) -> None:
        self.swerveModuleSims = swerveModuleSims
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            *(module.position for module in swerveModuleSims)
        )
        self.pose = constants.kSimDefaultRobotLocation
        self.outputs = None

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, robotVoltage: float) -> None:
        deltaT = tm_diff
        states = []
        for module in self.swerveModuleSims:
            wheelVoltage = module.wheelMotorSim.getSpeed() * robotVoltage
            wheelAngularVelocity = (
                wheelVoltage
                * module.wheelMotorType.Kv
                / module.driveMotorGearing  # scale the wheel motor to get more reasonable wheel speeds
            )
            wheelLinearVelocity = (
                wheelAngularVelocity * constants.kWheelDistancePerRadian
            )
            module.wheelEncoderSim.setRate(wheelLinearVelocity)

            deltaWheelDistance = wheelLinearVelocity * deltaT
            newWheelDistance = module.wheelEncoderSim.getDistance() + deltaWheelDistance
            module.wheelEncoderSim.setDistance(newWheelDistance)

            swerveVoltage = module.swerveMotorSim.getSpeed() * robotVoltage
            swerveAngularVelocity = (
                swerveVoltage
                * module.swerveMotorType.Kv
                / module.steerMotorGearing  # scale the swerve motor to get more reasonable swerve speeds
            )
            module.swerveEncoderSim.setRate(swerveAngularVelocity)

            deltaSwerveAngle = swerveAngularVelocity * deltaT
            newSwerveAngle = module.swerveEncoderSim.getDistance() + deltaSwerveAngle
            module.swerveEncoderSim.setDistance(newSwerveAngle)

            state = wpimath.kinematics.SwerveModuleState(
                wheelLinearVelocity,
                Rotation2d(newSwerveAngle),
            )
            states.append(state)

        chassisSpeed = self.kinematics.toChassisSpeeds(*states)
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


class VisionSim:
    def __init__(self) -> None:
        self.system = SimVisionSystem(
            constants.kPhotonvisionCameraName,
            constants.kPhotonvisionCameraDiagonalFOV,
            constants.kLimelightRelativeToRobotTransform.inverse(),
            9000,
            *constants.kPhotonvisionCameraPixelDimensions,
            0.1,
        )

        for tag_id, position in zip(
            constants.kApriltagPositionDict.keys(),
            constants.kApriltagPositionDict.values(),
        ):
            simTarget = SimVisionTarget(
                position, constants.kApriltagWidth, constants.kApriltagHeight, tag_id
            )
            self.system.addSimVisionTarget(simTarget)

    def update(self, robotPose: Pose2d) -> None:
        self.system.processFrame(robotPose)

class ArmSimulation:
    def __init__(
        self,
        shoulderSimMotor: TalonFXSimCollection,
        elbowSimMotor: TalonFXSimCollection,
        wristSimMotor: TalonFXSimCollection,
        shoulderSimEncoder: CANCoderSimCollection,
        elbowSimEncoder: CANCoderSimCollection,
        wristSimEncoder: CANCoderSimCollection,
    ) -> None:

        self.shoulderGearbox = DCMotor.falcon500(1)
        self.elbowGearbox = DCMotor.falcon500(1)
        self.wristGearbox = DCMotor.falcon500(1)

        self.elbowJointSim = SingleJointedArmSim(
            self.elbowGearbox,
            constants.kElbowArmGearRatio,
            SingleJointedArmSim.estimateMOI(
                constants.kArmelbowLength, constants.kArmelbowMass
            ),
            constants.kArmelbowLength,
            -180 / constants.kRadiansPerDegree,
            180 / constants.kRadiansPerDegree,
            constants.kArmelbowMass,
            False,
            [0],
        )
        self.shoulderJointSim = SingleJointedArmSim(
            self.shoulderGearbox,
            constants.kShoulderArmGearRatio,
            SingleJointedArmSim.estimateMOI(
                constants.kArmshoulderLength, constants.kArmshoulderMass
            ),
            constants.kArmshoulderLength,
            math.radians(-180),
            math.radians(180),
            constants.kArmshoulderMass,
            False,
            [0],
        )
        self.wristJointSim = SingleJointedArmSim(
            self.wristGearbox,
            constants.kWristArmGearRatio,
            SingleJointedArmSim.estimateMOI(
                constants.kArmwristLength, constants.kArmwristMass
            ),
            constants.kArmwristLength,
            math.radians(-180),
            math.radians(180),
            constants.kArmwristMass,
            False,
            [0],
        )

        self.shoulderSimMotor = shoulderSimMotor
        self.elbowSimMotor = elbowSimMotor
        self.wristSimMotor = wristSimMotor

        self.shoulderSimEncoder = shoulderSimEncoder
        self.elbowSimEncoder = elbowSimEncoder
        self.wristSimEncoder = wristSimEncoder

    def update(self, tm_diff) -> None:
        self.shoulderJointSim.setInput(
            0, self.shoulderSimMotor.getMotorOutputLeadVoltage()
        )
        self.elbowJointSim.setInput(0, self.elbowSimMotor.getMotorOutputLeadVoltage())
        self.wristJointSim.setInput(0, self.wristSimMotor.getMotorOutputLeadVoltage())

        self.shoulderJointSim.update(tm_diff)
        self.elbowJointSim.update(tm_diff)
        self.wristJointSim.update(tm_diff)

        self.elbowSimMotor.setIntegratedSensorRawPosition(
            int(
                (self.shoulderJointSim.getAngle() + self.elbowJointSim.getAngle())
                * constants.kTalonEncoderPulsesPerRadian
                * constants.kElbowArmGearRatio
            )  # convert relative to 4bar angles
        )
        self.elbowSimEncoder.setRawPosition(
            int(
                self.elbowJointSim.getAngle() * constants.kCANcoderPulsesPerRadian
            )  # convert relative to 4bar angles
        )

        self.shoulderSimMotor.setIntegratedSensorRawPosition(
            int(
                self.shoulderJointSim.getAngle()
                * constants.kTalonEncoderPulsesPerRadian
                * constants.kShoulderArmGearRatio
            )
        )
        self.shoulderSimEncoder.setRawPosition(
            int(self.shoulderJointSim.getAngle() * constants.kCANcoderPulsesPerRadian)
        )

        self.wristSimMotor.setIntegratedSensorRawPosition(
            int(
                (
                    self.shoulderJointSim.getAngle()
                    + self.elbowJointSim.getAngle()
                    + self.wristJointSim.getAngle()
                )  # convert relative to 4bar angles
                * constants.kTalonEncoderPulsesPerRadian
                * constants.kWristArmGearRatio
            )
        )
        self.wristSimEncoder.setRawPosition(
            int(self.wristJointSim.getAngle() * constants.kCANcoderPulsesPerRadian)
        )


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: "MentorBot"):
        self.physics_controller = physics_controller

        self.frontLeftModuleSim = SwerveModuleSim(
            constants.kFrontLeftWheelPosition,
            PWMSim(constants.kSimFrontLeftDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimFrontLeftSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimFrontLeftDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimFrontLeftSteerEncoderPorts[0]),
        )
        self.frontRightModuleSim = SwerveModuleSim(
            constants.kFrontRightWheelPosition,
            PWMSim(constants.kSimFrontRightDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimFrontRightSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimFrontRightDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimFrontRightSteerEncoderPorts[0]),
        )
        self.backSimLeftModule = SwerveModuleSim(
            constants.kBackLeftWheelPosition,
            PWMSim(constants.kSimBackLeftDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimBackLeftSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimBackLeftDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimBackLeftSteerEncoderPorts[0]),
        )
        self.backSimRightModule = SwerveModuleSim(
            constants.kBackRightWheelPosition,
            PWMSim(constants.kSimBackRightDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimBackRightSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimBackRightDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimBackRightSteerEncoderPorts[0]),
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backSimLeftModule,
            self.backSimRightModule,
        ]

        self.driveSim = SwerveDriveSim(tuple(self.swerveModuleSims))
        self.armSim = ArmSimulation(
            robot.container.arm.shoulderArm.getSimCollection(),
            robot.container.arm.elbowArm.getSimCollection(),
            robot.container.arm.wristArm.getSimCollection(),
            robot.container.arm.shoulderEncoder.getSimCollection(),
            robot.container.arm.elbowEncoder.getSimCollection(),
            robot.container.arm.wristEncoder.getSimCollection(),
        )

        self.gyroSim = SimDeviceSim("navX-Sensor[4]")
        self.gyroYaw = self.gyroSim.getDouble("Yaw")

        self.sim_initialized = False

        self.vision = VisionSim()

        targets = []
        for target in constants.kApriltagPositionDict.values():
            x = target.X()
            y = target.Y()
            z = target.Z()
            rotationQuaternion = target.rotation().getQuaternion()
            w_rot = rotationQuaternion.W()
            x_rot = rotationQuaternion.X()
            y_rot = rotationQuaternion.Y()
            z_rot = rotationQuaternion.Z()

            targets.append(
                [x, y, z, w_rot, x_rot, y_rot, z_rot]
            )  # https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md#cones

        SmartDashboard.putNumberArray(
            constants.kFieldSimTargetKey,
            functools.reduce(
                operator.add, targets, []
            ),  # adds all the values found within targets (converts [[]] to [])
        )

    # pylint: disable-next=unused-argument
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        if not self.sim_initialized:
            self.sim_initialized = True
            # self.physics_controller.field, is not set until simulation_init
            simTargetObject = self.physics_controller.field.getObject(
                constants.kSimTargetName
            )
            simTargetObject.setPose(constants.kSimDefaultTargetLocation)
            simBallObject = self.physics_controller.field.getObject(
                constants.kSimBallName
            )
            simBallObject.setPose(constants.kSimDefaultBallLocation)

        self.gyroYaw.set(self.driveSim.getHeading().degrees())

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.driveSim.update(tm_diff, voltage)
        self.armSim.update(tm_diff)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)

        # simulate the vision system
        self.vision.update(simRobotPose)

        # publish the simulated robot pose to nt
        SmartDashboard.putNumberArray(
            constants.kSimRobotPoseArrayKey,
            [simRobotPose.X(), simRobotPose.Y(), simRobotPose.rotation().radians()],
        )

        # publish the simulated target and ball pose to nt
        simTargetObject = self.physics_controller.field.getObject(
            constants.kSimTargetName
        )
        simTargetPose = simTargetObject.getPose()
        SmartDashboard.putNumberArray(
            constants.kSimTargetPoseArrayKey,
            [simTargetPose.X(), simTargetPose.Y(), simTargetPose.rotation().radians()],
        )
