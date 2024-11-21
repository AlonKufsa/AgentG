package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue.Signed_PlusMinusHalf
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.rps
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.sqrt

object SwerveConstants {
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

	const val WHEEL_RADIUS_METERS = 0.0508
	const val MAX_SPEED_MPS = 5.0
	val MAX_ANGULAR_VELOCITY = 2.0.rps
	val DRIVEBASE_RADIUS_METERS = 0.417405
	val MODULE_OFFSET = (DRIVEBASE_RADIUS_METERS / sqrt(2.0))

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

	fun canCoderConfigs(moduleName: String): CANcoderConfiguration = CANcoderConfiguration().apply {
		with(MagnetSensor) {
			// The offset added to the CANCoder for it to measure correctly for a wheel pointing right to be 0 degrees
			MagnetOffset = when (moduleName) {
				"FrontRight" -> FRONT_RIGHT_OFFSET.rotations
				"FrontLeft" -> FRONT_LEFT_OFFSET.rotations
				"BackLeft" -> BACK_LEFT_OFFSET.rotations
				"BackRight" -> BACK_RIGHT_OFFSET.rotations
				else -> 0.0.also { DriverStation.reportError("Invalid swerve module name: $moduleName", false) }
			}

			SensorDirection = CounterClockwise_Positive
			AbsoluteSensorRange = Signed_PlusMinusHalf
		}
	}

	val pigeonConfigs: Pigeon2Configuration = Pigeon2Configuration()
}