package frc.robot.commands

import com.hamosad1657.lib.commands.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import kotlin.math.pow

fun SwerveSubsystem.driveAngularVelocity(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rJoyXSupplier: () -> Double,
): Command = withName("Drive with angular velocity control") {
	run {
		val lJoyY = lJoyYSupplier().pow(3)
		val lJoyX = lJoyXSupplier().pow(3)
		val rJoyX = rJoyXSupplier().pow(3)

		val chassisSpeeds = ChassisSpeeds(
			-lJoyY * SwerveConstants.MAX_SPEED_MPS,
			-lJoyX * SwerveConstants.MAX_SPEED_MPS,
			-rJoyX * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs
		)
		drive(true, chassisSpeeds)
	} finallyDo {
		resetModules()
	}
}