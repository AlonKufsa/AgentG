package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.subsystems.swerve.SwerveSubsystem.PoseEstimationState.*
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveRotationControlState.ROTATION_SETPOINT
import frc.robot.RobotMap.SwerveMap as Map
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase("Swerve") {
	// FR, FL, BL, BR
	private val frontRight = SwerveModule(
		driveMotorID = Map.FrontRight.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = true,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.FrontRight.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.FrontRight.CANCODER_ID),
		invertedSteer = false,
		canCoderID = Map.FrontRight.CANCODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("FrontRight"),
		wheelRadius = Constants.WHEEL_RADIUS,
		moduleName = "FrontRight",
		Constants.SWERVE_CAN_BUS,
	)
	private val frontLeft = SwerveModule(
		driveMotorID = Map.FrontLeft.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = true,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.FrontLeft.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.FrontLeft.CANCODER_ID),
		invertedSteer = false,
		canCoderID = Map.FrontLeft.CANCODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("FrontLeft"),
		wheelRadius = Constants.WHEEL_RADIUS,
		moduleName = "FrontLeft",
		Constants.SWERVE_CAN_BUS,
	)
	private val backLeft = SwerveModule(
		driveMotorID = Map.BackLeft.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = true,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.BackLeft.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.BackLeft.CANCODER_ID),
		invertedSteer = false,
		canCoderID = Map.BackLeft.CANCODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("BackLeft"),
		wheelRadius = Constants.WHEEL_RADIUS,
		moduleName = "BackLeft",
		Constants.SWERVE_CAN_BUS,
	)
	private val backRight = SwerveModule(
		driveMotorID = Map.BackRight.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = true,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.BackRight.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.BackRight.CANCODER_ID),
		invertedSteer = false,
		canCoderID = Map.BackRight.CANCODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("BackRight"),
		wheelRadius = Constants.WHEEL_RADIUS,
		moduleName = "BackRight",
		Constants.SWERVE_CAN_BUS,
	)
	private val pigeon = Pigeon2(Map.PIGEON_2_ID, Constants.SWERVE_CAN_BUS).apply {
		configurator.apply(Constants.pigeonConfigs)
	}

	private val swerveKinematics = SwerveDriveKinematics(
		Translation2d(Constants.MODULE_OFFSET.asMeters, Constants.MODULE_OFFSET.asMeters),
		Translation2d(-Constants.MODULE_OFFSET.asMeters, Constants.MODULE_OFFSET.asMeters),
		Translation2d(-Constants.MODULE_OFFSET.asMeters, -Constants.MODULE_OFFSET.asMeters),
		Translation2d(Constants.MODULE_OFFSET.asMeters, -Constants.MODULE_OFFSET.asMeters),
	)
	private val poseEstimator = SwerveDrivePoseEstimator(
		swerveKinematics,
		angle,
		currentSwervePositionsArray,
		Pose2d(),
	)
	private val fieldWidget = Field2d()

	private val chassisRotationPID = PIDController(
		Constants.CHASSIS_ROTATION_SETPOINT_PID_GAINS.kP,
		Constants.CHASSIS_ROTATION_SETPOINT_PID_GAINS.kI,
		Constants.CHASSIS_ROTATION_SETPOINT_PID_GAINS.kD,
	).apply {
		enableContinuousInput(0.0, 360.0)
	}
	var rotationSetpoint = Rotation2d()

	private val angle: Rotation2d
		get() = Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.rotation2d.degrees, 0.0, 360.0))
	private val currentSwervePositionsArray: Array<SwerveModulePosition>
		get() = arrayOf(
			frontRight.currentPosition,
			frontLeft.currentPosition,
			backLeft.currentPosition,
			backRight.currentPosition,
		)
	private val robotRelativeSpeeds: ChassisSpeeds
		get() = SwerveKinematics.currentRobotRelativeChassisSpeeds
	private val fieldRelativeSpeeds: ChassisSpeeds
		get() = SwerveKinematics.currentFieldRelativeChassisSpeeds
	private val estimatedPose: Pose2d
		get() = poseEstimator.estimatedPosition


	enum class SwerveRotationControlState {
		ANGULAR_VELOCITY,
		ROTATION_SETPOINT, // Chassis speed's omega is ignored and an external rotation setpoint is used
	}

	var swerveRotationControlState = SwerveRotationControlState.ANGULAR_VELOCITY

	enum class PoseEstimationState {
		OFF,
		VISION_ONLY,
		ODOMETRY_ONLY,
		FULL_POSE_ESTIMATION,
	}

	var poseEstimationState = ODOMETRY_ONLY

	init {
		AutoBuilder.configureHolonomic(
			{ estimatedPose },
			this::resetOdometry,
			{ robotRelativeSpeeds },
			{ speeds: ChassisSpeeds -> drive(false, speeds) },
			Constants.PP_CONFIGS,
			{
				Robot.getAlliance() == Red
			},
			this,
		)
	}

	fun resetModules() {
		frontRight.setModuleState(SwerveModuleState())
		frontLeft.setModuleState(SwerveModuleState())
		backLeft.setModuleState(SwerveModuleState())
		backRight.setModuleState(SwerveModuleState())
	}

	// Gyro
	fun resetGyro() {
		val pose = estimatedPose
		pigeon.reset()
		poseEstimator.resetPosition(
			Rotation2d(),
			currentSwervePositionsArray,
			Pose2d(pose.x, pose.y, Rotation2d()),
		)
	}

	fun setGyro(newAngle: Rotation2d) {
		val pose = estimatedPose
		pigeon.setYaw(newAngle.degrees)
		poseEstimator.resetPosition(
			newAngle,
			currentSwervePositionsArray,
			Pose2d(pose.x, pose.y, newAngle),
		)
	}

	// Pose estimation
	fun resetOdometry(pose: Pose2d) {
		poseEstimator.resetPosition(angle, currentSwervePositionsArray, pose)
	}

	private fun applyVisionMeasurement() {
		// TODO: Implement
	}

	// Drive
	fun drive(fieldRelative: Boolean, chassisSpeeds: ChassisSpeeds) {
		chassisRotationPID.setpoint = rotationSetpoint.degrees
		if (swerveRotationControlState == ROTATION_SETPOINT) {
			chassisSpeeds.omegaRadiansPerSecond = chassisRotationPID.calculate(angle.degrees)
		}
		val moduleStates =
			if (fieldRelative) {
				SwerveKinematics.fieldRelativeChassisSpeedsToModuleStates(
					chassisSpeeds,
					angle,
					Constants.MAX_SPEED_MPS,
					Constants.DRIVEBASE_RADIUS.asMeters,
				)
			} else {
				SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(
					chassisSpeeds,
					Constants.MAX_SPEED_MPS,
					Constants.DRIVEBASE_RADIUS.asMeters,
				)
			}
		frontRight.setModuleState(moduleStates[0])
		frontLeft.setModuleState(moduleStates[1])
		backLeft.setModuleState(moduleStates[2])
		backRight.setModuleState(moduleStates[3])
	}

	// Periodic
	override fun periodic() {
		// Pose estimation
		if (poseEstimationState == FULL_POSE_ESTIMATION || poseEstimationState == ODOMETRY_ONLY) {
			poseEstimator.update(
				angle,
				currentSwervePositionsArray,
			)
		}
		if (poseEstimationState == FULL_POSE_ESTIMATION || poseEstimationState == VISION_ONLY) {
			applyVisionMeasurement()
		}
		fieldWidget.robotPose = estimatedPose
	}

	// Telemetry
	override fun initSendable(builder: SendableBuilder) {
		frontRight.sendModuleInfo(builder)
		frontLeft.sendModuleInfo(builder)
		backLeft.sendModuleInfo(builder)
		backRight.sendModuleInfo(builder)

		builder.addDoubleProperty("Robot heading deg", { angle.degrees }, null)

		SmartDashboard.putData(fieldWidget)
	}
}