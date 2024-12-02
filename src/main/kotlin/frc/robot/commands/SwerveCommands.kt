package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveRotationControlState.ANGULAR_VELOCITY
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveRotationControlState.ROTATION_SETPOINT
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
	runOnce {
		swerveRotationControlState = ROTATION_SETPOINT
	} andThen run {
		val lJoyY = lJoyYSupplier().pow(3)
		val lJoyX = lJoyXSupplier().pow(3)

		rotationSetpoint = rotationSetpointInput()

		val chassisSpeeds = ChassisSpeeds(
			lJoyY * SwerveConstants.MAX_SPEED_MPS,
			-lJoyX * SwerveConstants.MAX_SPEED_MPS,
			0.0,
		)
		drive(fieldRelative, chassisSpeeds)
	} finallyDo {
		resetModules()
	}
}

fun SwerveSubsystem.followPathCommand(path: PathPlannerPath): Command = AutoBuilder.followPath(path)

fun SwerveSubsystem.followPathWithRotationSetpointCommand(
	path: PathPlannerPath,
	rotationSetpointInput: () -> Rotation2d,
): Command {
	swerveRotationControlState = ROTATION_SETPOINT
	return withName("Follow path with rotation setpoint") {
		ParallelCommandGroup(
			run { rotationSetpoint = rotationSetpointInput() },
			followPathCommand(path),
		) finallyDo {
			swerveRotationControlState = ANGULAR_VELOCITY
		}
	}
}

fun SwerveSubsystem.resetOdometryCommand(pose: Pose2d): Command = withName("Reset odometry") {
	runOnce { resetOdometry(pose) }
}

fun SwerveSubsystem.resetGyroCommand(): Command = withName("Reset gyro") {
	runOnce { resetGyro() }
}