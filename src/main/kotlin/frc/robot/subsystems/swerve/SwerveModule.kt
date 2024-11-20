package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.PI

/** Drive motor ID - The ID of the drive motor.
 *
 * Drive motor configs - The talonFX configs of your drive motor.
 *
 * Inverted drive - Weather the drive motor should have it's movement inverted.
 *
 * Drive transmission - The amount of turns the drive motor does for every wheel rotation
 *
 *
 * Steer motor ID - The ID of the steer motor.
 *
 * Steer motor configs - The talonFX configs of your steer motor.
 *
 * Inverted steer - Weather the steer motor should have it's movement inverted.
 *
 *
 * CAN coder ID - The ID of the wheel angle CAN coder.
 *
 * CAN coder configs - the configs of the wheel angle CAN coder.
 *  */
class SwerveModule(
	private val driveMotorID: Int,
	private val driveMotorConfigs: TalonFXConfiguration,
	private val invertedDrive: Boolean = false,
	private val driveTransmission: Double,

	private val steerMotorID: Int,
	private val steerMotorConfigs: TalonFXConfiguration,
	private val invertedSteer: Boolean = false,

	private val canCoderID: Int,
	private val canCoderConfigs: CANcoderConfiguration,

	private val wheelRadiusMeters: Double,

) {
	private val driveMotor = HaTalonFX(driveMotorID).apply {
		inverted = invertedDrive
		configurator.apply(driveMotorConfigs)
	}

	private val steerMotor = HaTalonFX(steerMotorID).apply {
		inverted = invertedSteer
		configurator.apply(steerMotorConfigs)
	}

	private val canCoder = CANcoder(canCoderID).apply {
		configurator.apply(canCoderConfigs)
	}

	private val wheelCircumferenceMeters = wheelRadiusMeters * 2 * PI

	val angle: Rotation2d get() = Rotation2d.fromDegrees(canCoder.absolutePosition.value)
	val speedMPS: Double get() = wheelCircumferenceMeters * driveMotor.velocity.value / driveTransmission
	val position: Double get() = wheelCircumferenceMeters * driveMotor.position.value / driveTransmission

	private var angleSetpoint: Rotation2d = Rotation2d()
	private set(value) {
		controlRequestSteerAngle.Position = value.rotations
		steerMotor.setControl(controlRequestSteerAngle)
		field = value
	}
	private var speedSetpointMPS: Double = 0.0
	set(value) {
		angularVelocitySetpoint = AngularVelocity.fromRps(value / wheelCircumferenceMeters * driveTransmission)
		field = value
	}
	private var angularVelocitySetpoint = AngularVelocity.fromRps(0.0)
		private set(value) {
			controlRequestDriveVelocity.Velocity = value.asRps
			driveMotor.setControl(controlRequestDriveVelocity)
			field = value
		}

	private var controlRequestDriveVelocity = MotionMagicVelocityVoltage(0.0)
	private var controlRequestSteerAngle = MotionMagicVoltage(0.0).apply {
		EnableFOC = true
		Slot = 0
		LimitForwardMotion = false
		LimitReverseMotion = false
	}
}