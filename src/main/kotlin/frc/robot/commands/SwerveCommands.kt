package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import java.util.Optional
import kotlin.math.pow

fun SwerveSubsystem.angularVelocityDriveCommand(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rJoyXSupplier: () -> Double,
	fieldRelative: Boolean,
): Command = withName("Drive with angular velocity control") {
	run {
		val lJoyY = lJoyYSupplier().pow(3)
		val lJoyX = lJoyXSupplier().pow(3)
		val rJoyX = rJoyXSupplier().pow(3)

		val chassisSpeeds = ChassisSpeeds(
			lJoyY * SwerveConstants.MAX_SPEED_MPS,
			-lJoyX * SwerveConstants.MAX_SPEED_MPS,
			rJoyX * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs,
		)
		drive(fieldRelative, chassisSpeeds)
	} finallyDo {
		resetModules()
	}
}

fun SwerveSubsystem.rotationSetpointDriveCommand(
	lJoyYSupplier: () -> Double,
	lJoyXSupplier: () -> Double,
	rotationSetpointInput: () -> Rotation2d,
	fieldRelative: Boolean,
): Command = withName("Drive with angular velocity control") {
	run {
		val lJoyY = lJoyYSupplier().pow(3)
		val lJoyX = lJoyXSupplier().pow(3)

		rotationSetpoint = rotationSetpointInput()

		val chassisSpeeds = ChassisSpeeds(
			lJoyY * SwerveConstants.MAX_SPEED_MPS,
			-lJoyX * SwerveConstants.MAX_SPEED_MPS,
			0.0,
		)
		driveRotationSetpoint(
			fieldRelative,
			chassisSpeeds.vxMetersPerSecond,
			chassisSpeeds.vyMetersPerSecond,
		)
	} finallyDo {
		resetModules()
	}
}

fun SwerveSubsystem.followPathCommand(path: PathPlannerPath): Command = AutoBuilder.followPath(path)

fun SwerveSubsystem.followPathWithRotationSetpointCommand(
	path: PathPlannerPath,
	rotationSetpointInput: () -> Rotation2d,
): Command {
	PPHolonomicDriveController.setRotationTargetOverride {
		Optional.of(rotationSetpointInput())
	}
	return withName("Follow path with rotation setpoint") {
		followPathCommand(path)
	} finallyDo {
		PPHolonomicDriveController.setRotationTargetOverride {
			Optional.ofNullable(null)
		}
	}
}

fun SwerveSubsystem.resetOdometryCommand(pose: Pose2d): Command = withName("Reset odometry") {
	runOnce { resetPoseEstimation(pose) }
}

fun SwerveSubsystem.resetGyroCommand(): Command = withName("Reset gyro") {
	runOnce { resetGyro() }
}