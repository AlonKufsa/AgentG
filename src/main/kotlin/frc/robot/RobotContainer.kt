package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.commands.*
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
	const val DEADBAND = 0.06
	val mainController = CommandPS4Controller(0)

	init {
		sendSubsystemData()
		configureDefaultCommands()
		configureBindings()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {
		mainController.options().onTrue(SwerveSubsystem.resetGyroCommand())
	}

	private fun configureDefaultCommands() {
//		SwerveSubsystem.defaultCommand = SwerveSubsystem.angularVelocityDriveCommand(
//			{ simpleDeadband(mainController.leftY, DEADBAND) },
//			{ simpleDeadband(mainController.leftX, DEADBAND) },
//			{ simpleDeadband(mainController.rightX, DEADBAND) },
//			true,
//		)
	}

	private fun sendSubsystemData() {
		SmartDashboard.putData(SwerveSubsystem)
	}

	fun getAutonomousCommand(): Command? {
		// TODO: Implement properly
		return null
	}
}