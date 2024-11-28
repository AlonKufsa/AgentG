package frc.robot.commands

import com.hamosad1657.lib.commands.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveRotationControlState.ANGULAR_VELOCITY
import kotlin.math.pow

fun SwerveSubsystem.angularVelocityDriveCommand(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rJoyXSupplier: () -> Double,
	fieldRelative: Boolean,
): Command = withName("Drive with angular velocity control") {
	runOnce {
		swerveRotationControlState = ANGULAR_VELOCITY
	} andThen run {
		val lJoyY = lJoyYSupplier().pow(3)
		val lJoyX = lJoyXSupplier().pow(3)
		val rJoyX = rJoyXSupplier().pow(3)

		val chassisSpeeds = ChassisSpeeds(
			lJoyY * SwerveConstants.MAX_SPEED_MPS,
			-lJoyX * SwerveConstants.MAX_SPEED_MPS,
			rJoyX * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs
		)
		drive(fieldRelative, chassisSpeeds)
	} finallyDo {
		resetModules()
	}
}

fun SwerveSubsystem.resetGyroCommand(): Command = withName("Reset gyro") {
	runOnce { resetGyro() }
}