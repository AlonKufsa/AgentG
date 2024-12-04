package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue.Signed_PlusMinusHalf
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Length
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.PI
import kotlin.math.sqrt

enum class PoseEstimationMode {
	OFF,
	VISION_ONLY,
	ODOMETRY_ONLY,
	FULL_POSE_ESTIMATION,
}

object SwerveConstants {
	enum class ModuleOffset(val offset: Rotation2d) {
		FRONT_RIGHT(Rotation2d.fromDegrees(0.0)),
		FRONT_LEFT(Rotation2d.fromDegrees(0.0)),
		BACK_LEFT(Rotation2d.fromDegrees(0.0)),
		BACK_RIGHT(Rotation2d.fromDegrees(0.0)),
	}

	const val SWERVE_CAN_BUS = "SwerveBus"

	val DRIVE_PID_GAINS = PIDGains(kP = 0.09)
	const val DRIVE_KA = 0.0
	const val DRIVE_KV = 0.12
	const val DRIVE_KS = 0.0
	const val DRIVE_MM_ACCELERATION = 100.0
	const val DRIVE_MM_CRUISE_VELOCITY = 100.0

	val STEER_PID_GAINS = PIDGains(kP = 40.0)
	const val STEER_KA = 0.0
	const val STEER_KV = 0.0
	const val STEER_KS = 0.0
	const val STEER_MM_ACCELERATION = 100.0
	const val STEER_MM_CRUISE_VELOCITY = 100.0

	val CHASSIS_ROTATION_SETPOINT_PID_GAINS = PIDGains(
		kP = 0.0,
		kI = 0.0,
		kD = 0.0,
	)

	val WHEEL_RADIUS = Length.fromMeters(0.0508)
	const val MAX_SPEED_MPS = 5.0

	/** The amount of rotations the motor does for every rotation of the wheel */
	const val DRIVE_TRANSMISSION = 6.746031746031747
	val DRIVEBASE_RADIUS = Length.fromMeters(0.417405)
	val MAX_ANGULAR_VELOCITY = AngularVelocity.fromRps(MAX_SPEED_MPS / 2 * PI * DRIVEBASE_RADIUS.asMeters)
	val MODULE_OFFSET = (DRIVEBASE_RADIUS / sqrt(2.0))

	val FRONT_RIGHT_OFFSET = Rotation2d.fromDegrees(-268.066406)
	val FRONT_LEFT_OFFSET = Rotation2d.fromDegrees(-222.539062)
	val BACK_LEFT_OFFSET = Rotation2d.fromDegrees(95.537)
	val BACK_RIGHT_OFFSET = Rotation2d.fromDegrees(-185.888672)

	val DRIVE_MOTOR_CONFIGS: TalonFXConfiguration = TalonFXConfiguration().apply {
		// Current limits
		CurrentLimits.SupplyCurrentLimit = 45.0
		CurrentLimits.SupplyCurrentLimitEnable = true
		CurrentLimits.StatorCurrentLimitEnable = false

		// Limits the speed in which the motors voltage consumption can change
		ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25

		// PID and FF
		with(Slot0) {
			// PID
			kP = DRIVE_PID_GAINS.kP
			kI = DRIVE_PID_GAINS.kI
			kD = DRIVE_PID_GAINS.kD

			// FF
			kA = DRIVE_KA
			kV = DRIVE_KV
			kS = DRIVE_KS
		}

		// MotionMagic configs
		with(MotionMagic) {
			MotionMagicAcceleration = DRIVE_MM_ACCELERATION
			MotionMagicCruiseVelocity = DRIVE_MM_CRUISE_VELOCITY
		}

	}

	fun steerMotorConfigs(canCoderID: Int): TalonFXConfiguration = TalonFXConfiguration().apply {
		CurrentLimits.SupplyCurrentLimit = 20.0
		CurrentLimits.SupplyCurrentLimitEnable = true
		CurrentLimits.StatorCurrentLimitEnable = false

		ClosedLoopGeneral.ContinuousWrap = true

		Feedback.FeedbackSensorSource = RemoteCANcoder
		Feedback.FeedbackRemoteSensorID = canCoderID

		with(Slot0) {
			// PID
			kP = STEER_PID_GAINS.kP
			kI = STEER_PID_GAINS.kI
			kD = STEER_PID_GAINS.kD

			// FF
			kA = STEER_KA
			kV = STEER_KV
			kS = STEER_KS
		}

		// Motion Magic configs
		with(MotionMagic) {
			MotionMagicAcceleration = STEER_MM_ACCELERATION
			MotionMagicCruiseVelocity = STEER_MM_CRUISE_VELOCITY
		}

	}

	fun canCoderConfigs(moduleOffset: ModuleOffset): CANcoderConfiguration = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			// The offset added to the CANCoder for it to measure correctly for a wheel pointing right to be 0 degrees
			MagnetOffset = moduleOffset.offset.rotations

			SensorDirection = CounterClockwise_Positive
			AbsoluteSensorRange = Signed_PlusMinusHalf
		}
	}

	val pigeonConfigs: Pigeon2Configuration = Pigeon2Configuration()

	val TRANSLATION_PID_GAINS = PIDConstants(0.2, 0.0, 0.0)
	val ROTATION_PID_GAINS = PIDConstants(0.1, 0.0, 0.0)
	val PP_CONFIGS = HolonomicPathFollowerConfig(
		TRANSLATION_PID_GAINS,
		ROTATION_PID_GAINS,
		MAX_SPEED_MPS,
		DRIVEBASE_RADIUS.asMeters,
		ReplanningConfig(),
	)
}