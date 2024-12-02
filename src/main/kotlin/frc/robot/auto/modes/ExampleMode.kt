package frc.robot.auto.modes

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.*
import frc.robot.subsystems.swerve.SwerveSubsystem

fun exampleModeAutoCommand(): Command = withName("ExampleMode auto") {
	SequentialCommandGroup(
		SwerveSubsystem.followPathCommand(PathPlannerPath.fromPathFile("Example Path")),
		SwerveSubsystem.followPathWithRotationSetpointCommand(
			PathPlannerPath.fromPathFile("Example Path 2")) { Rotation2d.fromDegrees(90.0) },
	)
}