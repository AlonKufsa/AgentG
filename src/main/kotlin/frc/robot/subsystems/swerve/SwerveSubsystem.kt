package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.hamosad1657.lib.robotPrint
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.MathUtil
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
import frc.robot.subsystems.swerve.PoseEstimationMode.*
import frc.robot.subsystems.swerve.SwerveConstants.ModuleOffset.*
import frc.robot.RobotMap.SwerveMap as Map
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase("Swerve") {
	init {
		AutoBuilder.configureHolonomic(
			{ estimatedPose },
			this::resetPoseEstimation,
			{ robotRelativeSpeeds },
			{ speeds: ChassisSpeeds -> drive(false, speeds) },
			Constants.PP_CONFIGS,
			{ Robot.getAlliance() == Red },
			this,
		)
	}

	var poseEstimationState = ODOMETRY_ONLY
		set(value) {
			if (field != value) {
				robotPrint("poseEstimation state changed to: ${value.name}")
			}
			field = value
		}
	var rotationSetpoint = Rotation2d()


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
		canCoderConfigs = Constants.canCoderConfigs(FRONT_RIGHT),
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
		canCoderConfigs = Constants.canCoderConfigs(FRONT_LEFT),
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
		canCoderConfigs = Constants.canCoderConfigs(BACK_LEFT),
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
		canCoderConfigs = Constants.canCoderConfigs(BACK_RIGHT),
		wheelRadius = Constants.WHEEL_RADIUS,
		moduleName = "BackRight",
		Constants.SWERVE_CAN_BUS,
	)
	private val modules = arrayOf(
		frontRight,
		frontLeft,
		backLeft,
		backRight,
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
		gyroAngle,
		currentModulePositions,
		Pose2d(),
	)
	private val fieldWidget = Field2d()

	/**
	 * Uses degrees
	 */
	private val chassisRotationPID = Constants.CHASSIS_ROTATION_SETPOINT_PID_GAINS.toPIDController().apply {
		enableContinuousInput(0.0, 360.0)
	}

	private val gyroAngle: Rotation2d
		get() = Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.rotation2d.degrees, 0.0, 360.0))
	private val currentModulePositions: Array<SwerveModulePosition>
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


	fun resetModules() {
		for (module in modules) {
			module.setModuleState(SwerveModuleState())
		}
	}

	// Gyro
	fun zeroGyro() {
		setGyro(Rotation2d())
	}

	fun setGyro(newAngle: Rotation2d) {
		val pose = estimatedPose
		pigeon.setYaw(newAngle.degrees)
		poseEstimator.resetPosition(
			newAngle,
			currentModulePositions,
			Pose2d(pose.x, pose.y, newAngle),
		)
	}

	// Pose estimation
	private fun updateOdometry() {
		poseEstimator.update(
			gyroAngle,
			currentModulePositions,
		)
	}

	fun resetPoseEstimation(newPose: Pose2d) {
		poseEstimator.resetPosition(gyroAngle, currentModulePositions, newPose)
	}

	private fun applyVisionMeasurement() {
		// TODO: Implement
	}

	// Drive
	fun drive(fieldRelative: Boolean, chassisSpeeds: ChassisSpeeds) {
		val moduleStates =
			if (fieldRelative) {
				SwerveKinematics.fieldRelativeChassisSpeedsToModuleStates(
					chassisSpeeds,
					gyroAngle,
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
		for (iN in 0..1) {
			modules[iN].setModuleState(moduleStates[iN])
		}
	}

	fun driveRotationSetpoint(fieldRelative: Boolean, vxMPS: Double, vyMPS: Double) {
		val chassisSpeeds = ChassisSpeeds(
			vxMPS,
			vyMPS,
			chassisRotationPID.calculate(gyroAngle.degrees, rotationSetpoint.degrees),
		)
		drive(fieldRelative, chassisSpeeds)
	}

	// Periodic
	override fun periodic() {
		// Pose estimation
		when (poseEstimationState) {
			ODOMETRY_ONLY -> {
				updateOdometry()
			}

			VISION_ONLY -> {
				applyVisionMeasurement()
			}

			FULL_POSE_ESTIMATION -> {
				updateOdometry()
				applyVisionMeasurement()
			}

			else -> {}
		}
		fieldWidget.robotPose = estimatedPose
	}

	// Telemetry
	override fun initSendable(builder: SendableBuilder) {
		for (module in modules) {
			module.sendModuleInfo(builder)
		}

		builder.addDoubleProperty("Robot heading deg", { gyroAngle.degrees }, null)

		SmartDashboard.putData(fieldWidget)
	}
}