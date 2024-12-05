package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.max

/** All module states are FR, FL, BL, BR */
object SwerveKinematics {

	var currentRobotRelativeChassisSpeeds = ChassisSpeeds()
	var currentFieldRelativeChassisSpeeds = ChassisSpeeds()

	/**
	 * Converts between angular velocity of the robot to the module states needed to achieve it.
	 *
	 * Positive angular velocity is counterclockwise, negative is clockwise.
	 */
	fun angularVelocityToModuleStates(
		angularVelocity: AngularVelocity,
		driveBaseRadiusMeters: Double,
	): Array<SwerveModuleState> {
		val wheelSpeedMPS: Double = angularVelocity.asRadPs * driveBaseRadiusMeters

		val moduleStates = Array<SwerveModuleState>(4) { SwerveModuleState() }

		moduleStates[0].angle = Rotation2d.fromDegrees(135.0)
		moduleStates[1].angle = Rotation2d.fromDegrees(-135.0)
		moduleStates[2].angle = Rotation2d.fromDegrees(-45.0)
		moduleStates[3].angle = Rotation2d.fromDegrees(45.0)
		for (moduleState in moduleStates) {
			moduleState.speedMetersPerSecond = wheelSpeedMPS
		}
		return moduleStates
	}

	/** Converts between a velocity in some direction to the module states needed to achieve it. */
	private fun robotRelativeVelocityToModuleStates(velocity: Translation2d): Array<SwerveModuleState> {
		return Array(4) { SwerveModuleState(velocity.norm, velocity.angle) }
	}

	private fun moduleStateToTranslation2d(moduleState: SwerveModuleState): Translation2d {
		return Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
	}

	/**
	 * Converts between chassis speeds and module states.
	 * Positive chassis speeds omega results in counterclockwise rotation.
	 *
	 * x direction is forward, y direction is left
	 */
	fun robotRelativeChassisSpeedsToModuleStates(
		chassisSpeeds: ChassisSpeeds,
		maxSpeedMPS: Double,
		driveBaseRadiusMeters: Double,
	): Array<SwerveModuleState> {
		val discreteChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02)
		currentRobotRelativeChassisSpeeds = discreteChassisSpeeds

		val moduleTranslationVelocity =
			Translation2d(-discreteChassisSpeeds.vyMetersPerSecond, discreteChassisSpeeds.vxMetersPerSecond)

		val velocityModuleState = robotRelativeVelocityToModuleStates(moduleTranslationVelocity)
		val rotationModuleStates =
			angularVelocityToModuleStates(AngularVelocity.fromRadPs(discreteChassisSpeeds.omegaRadiansPerSecond),
				driveBaseRadiusMeters)

		val wantedStates = Array(4) { SwerveModuleState() }

		for (iN in 0..3) {
			val combinedTranslation =
				moduleStateToTranslation2d(velocityModuleState[iN]) + moduleStateToTranslation2d(rotationModuleStates[iN])
			wantedStates[iN].angle = combinedTranslation.angle
			wantedStates[iN].speedMetersPerSecond = combinedTranslation.norm
		}

		SmartDashboard.putNumberArray("Wanted module states", arrayOf(
			wantedStates[0].angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, wantedStates[0].speedMetersPerSecond,
			wantedStates[1].angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, wantedStates[1].speedMetersPerSecond,
			wantedStates[2].angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, wantedStates[2].speedMetersPerSecond,
			wantedStates[3].angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, wantedStates[3].speedMetersPerSecond,
		))
		return factorModuleStates(maxSpeedMPS, wantedStates)
	}

	/** Heading is counter-clockwise positive.
	 *
	 * Chassis speeds have x represent the direction going away from the blue driver station, and y to the left of it
	 *
	 * Positive omega results in robot spinning counter-clockwise*/
	fun fieldRelativeChassisSpeedsToModuleStates(
		chassisSpeeds: ChassisSpeeds,
		heading: Rotation2d,
		maxSpeedMPS: Double,
		driveBaseRadiusMeters: Double,
	): Array<SwerveModuleState> {
		val discreteChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02)
		currentFieldRelativeChassisSpeeds = discreteChassisSpeeds

		val moduleTranslationVelocity =
			Translation2d(discreteChassisSpeeds.vxMetersPerSecond, discreteChassisSpeeds.vyMetersPerSecond).rotateBy(
				heading)
		val robotRelativeSpeeds = ChassisSpeeds(
			moduleTranslationVelocity.x,
			moduleTranslationVelocity.y,
			discreteChassisSpeeds.omegaRadiansPerSecond
		)
		return robotRelativeChassisSpeedsToModuleStates(robotRelativeSpeeds, maxSpeedMPS, driveBaseRadiusMeters)
	}

	private fun factorModuleStates(
		maxSpeedMPS: Double,
		moduleStates: Array<SwerveModuleState>,
	): Array<SwerveModuleState> {
		val highestVelocity = max(max(moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond),
			max(moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond))
		if (highestVelocity > maxSpeedMPS && maxSpeedMPS != 0.0) {

			val factor = maxSpeedMPS / highestVelocity

			for (state in moduleStates) {
				state.speedMetersPerSecond *= factor
			}
		}
		return moduleStates
	}
}