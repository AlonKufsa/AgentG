package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.SwerveMap as Map
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase("Swerve") {
	// FR, FL, BL, BR
	private val frontRight = SwerveModule(
		driveMotorID = Map.FrontRight.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = false,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.FrontRight.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.FrontRight.STEER_MOTOR_ID),
		invertedSteer = false,
		canCoderID = Map.FrontRight.CAN_CODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("FrontRight"),
		wheelRadiusMeters = Constants.WHEEL_RADIUS_METERS,
		moduleName = "FrontRight"
	)
	private val frontLeft = SwerveModule(
		driveMotorID = Map.FrontLeft.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = false,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.FrontLeft.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.FrontLeft.STEER_MOTOR_ID),
		invertedSteer = false,
		canCoderID = Map.FrontLeft.CAN_CODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("FrontLeft"),
		wheelRadiusMeters = Constants.WHEEL_RADIUS_METERS,
		moduleName = "FrontLeft"
	)
	private val backLeft = SwerveModule(
		driveMotorID = Map.BackLeft.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = false,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.BackLeft.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.BackLeft.STEER_MOTOR_ID),
		invertedSteer = false,
		canCoderID = Map.BackLeft.CAN_CODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("BackLeft"),
		wheelRadiusMeters = Constants.WHEEL_RADIUS_METERS,
		moduleName = "BackLeft"
	)
	private val backRight = SwerveModule(
		driveMotorID = Map.BackRight.DRIVE_MOTOR_ID,
		driveMotorConfigs = Constants.DRIVE_MOTOR_CONFIGS,
		invertedDrive = false,
		driveTransmission = Constants.DRIVE_TRANSMISSION,
		steerMotorID = Map.BackRight.STEER_MOTOR_ID,
		steerMotorConfigs = Constants.steerMotorConfigs(Map.BackRight.STEER_MOTOR_ID),
		invertedSteer = false,
		canCoderID = Map.BackRight.CAN_CODER_ID,
		canCoderConfigs = Constants.canCoderConfigs("BackRight"),
		wheelRadiusMeters = Constants.WHEEL_RADIUS_METERS,
		moduleName = "BackRight"
	)
	private val pigeon = Pigeon2(Map.PIGEON_2_ID).apply {
		configurator.apply(Constants.pigeonConfigs)
	}

	private val swerveKinematics = SwerveDriveKinematics(
		Translation2d(Constants.MODULE_OFFSET, Constants.MODULE_OFFSET),
		Translation2d(-Constants.MODULE_OFFSET, Constants.MODULE_OFFSET),
		Translation2d(-Constants.MODULE_OFFSET, -Constants.MODULE_OFFSET),
		Translation2d(Constants.MODULE_OFFSET, -Constants.MODULE_OFFSET)
	)
	private val poseEstimator = SwerveDrivePoseEstimator(
		swerveKinematics,
		angle,
		currentSwervePositionsArray,
		Pose2d()
	)

	private val angle: Rotation2d
		get() = pigeon.rotation2d
	private val currentSwervePositionsArray: Array<SwerveModulePosition>
		get() = arrayOf(frontRight.position, frontLeft.position, backLeft.position, backRight.position)

	enum class SwerveControlState {
		IDLE,
		FULL_MANUAL_CONTROL,
		FOLLOWING_PATH,
		FOLLOWING_PATH_TRANSLATION_ONLY,
		
	}
}