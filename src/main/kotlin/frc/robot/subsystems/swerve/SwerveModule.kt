package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Length
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.PI

/**
 * @param driveMotorID - The ID of the drive motor.
 * @param driveMotorConfigs - The talonFX configs of your drive motor.
 * @param invertedDrive - Whether the drive motor should have it's movement inverted.
 * @param driveTransmission - The amount of turns the drive motor does for every wheel rotation
 *
 * @param steerMotorID - The ID of the steer motor.
 * @param steerMotorConfigs - The talonFX configs of your steer motor.
 * @param invertedSteer - Weather the steer motor should have it's movement inverted.
 *
 * @param canCoderID - The ID of the wheel angle CAN coder.
 * @param canCoderConfigs - the configs of the wheel angle CAN coder.
 *
 * @param wheelRadius - The radius of the wheel
 * @param moduleName - The name of the module (FrontRight, BackLeft...)
 * @param canBusName - The name of the CANBus network used for the module
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

	wheelRadius: Length,
	private val moduleName: String,
	private val canBusName: String,
) {
	private val driveMotor = HaTalonFX(driveMotorID, canBusName).apply {
		restoreFactoryDefaults()
		inverted = invertedDrive
		configurator.apply(driveMotorConfigs)
	}

	private val steerMotor = HaTalonFX(steerMotorID, canBusName).apply {
		restoreFactoryDefaults()
		inverted = invertedSteer
		configurator.apply(steerMotorConfigs)
	}

	private val canCoder = CANcoder(canCoderID, canBusName).apply {
		configurator.apply(canCoderConfigs)
	}

	private val wheelCircumference = wheelRadius * 2.0 * PI

	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(canCoder.absolutePosition.value)
	val currentSpeedMPS: Double get() = wheelCircumference.asMeters * driveMotor.velocity.value / driveTransmission
	val currentPosition: SwerveModulePosition
		get() = SwerveModulePosition(wheelCircumference.asMeters * driveMotor.position.value / driveTransmission,
			currentAngle)

	private var angleSetpoint: Rotation2d = Rotation2d()
	private var speedSetpointMPS: Double = 0.0

	private var controlRequestDriveVelocity = MotionMagicVelocityVoltage(0.0)
	private var controlRequestSteerAngle = MotionMagicVoltage(0.0).apply {
		Slot = 0
		LimitForwardMotion = false
		LimitReverseMotion = false
	}


	private fun setModuleSpeedMPS(speedMPS: Double) {
		speedSetpointMPS = speedMPS
		controlRequestDriveVelocity.Velocity = speedMPS / wheelCircumference.asMeters * driveTransmission
		driveMotor.setControl(controlRequestDriveVelocity)
	}

	private fun setModuleAngle(angle: Rotation2d) {
		angleSetpoint = angle
		controlRequestSteerAngle.Position = angle.rotations
		steerMotor.setControl(controlRequestSteerAngle)
	}

	fun setModuleState(state: SwerveModuleState) {
		setModuleSpeedMPS(state.speedMetersPerSecond)
		setModuleAngle(state.angle)
	}

	// Logging
	fun sendModuleInfo(builder: SendableBuilder) {
		builder.addDoubleProperty("$moduleName rotation deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("$moduleName speed MPS", { currentSpeedMPS }, null)

		builder.addDoubleProperty("$moduleName rotation setpoint deg", { angleSetpoint.degrees }, null)
		builder.addDoubleProperty("$moduleName speed setpoint MPS", { speedSetpointMPS }, null)
	}
}