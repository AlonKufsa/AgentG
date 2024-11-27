package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI
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
		val wheelSpeedMPS: Double = angularVelocity.asRps * 2 * PI * driveBaseRadiusMeters

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
	 */
	fun robotRelativeChassisSpeedsToModuleStates(
		chassisSpeeds: ChassisSpeeds,
		maxSpeedMPS: Double,
		driveBaseRadiusMeters: Double,
	): Array<SwerveModuleState> {
		val discreteChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02)
		currentRobotRelativeChassisSpeeds = discreteChassisSpeeds

		val moduleTranslationVelocity =
			Translation2d(discreteChassisSpeeds.vyMetersPerSecond, discreteChassisSpeeds.vxMetersPerSecond)

		val velocityModuleState = robotRelativeVelocityToModuleStates(moduleTranslationVelocity)
		val rotationModuleStates =
			angularVelocityToModuleStates(AngularVelocity.fromRadPs(discreteChassisSpeeds.omegaRadiansPerSecond),
				driveBaseRadiusMeters)

		val wantedStates = Array(4) { SwerveModuleState() }

		val frontRightCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleState[0]) + moduleStateToTranslation2d(rotationModuleStates[0])
		wantedStates[0].angle = frontRightCombined.angle
		wantedStates[0].speedMetersPerSecond = frontRightCombined.norm

		val frontLeftCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleState[1]) + moduleStateToTranslation2d(rotationModuleStates[1])
		wantedStates[1].angle = frontLeftCombined.angle
		wantedStates[1].speedMetersPerSecond = frontLeftCombined.norm

		val backLeftCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleState[2]) + moduleStateToTranslation2d(rotationModuleStates[2])
		wantedStates[2].angle = backLeftCombined.angle
		wantedStates[2].speedMetersPerSecond = backLeftCombined.norm

		val backRightCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleState[3]) + moduleStateToTranslation2d(rotationModuleStates[3])
		wantedStates[3].angle = backRightCombined.angle
		wantedStates[3].speedMetersPerSecond = backRightCombined.norm

		SmartDashboard.putNumberArray("Wanted module states", arrayOf(
			frontRightCombined.angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, frontRightCombined.norm,
			frontLeftCombined.angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, frontLeftCombined.norm,
			backLeftCombined.angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, backLeftCombined.norm,
			backRightCombined.angle.rotateBy(Rotation2d.fromDegrees(-90.0)).degrees, backRightCombined.norm
		))
		return factorModuleStates(maxSpeedMPS, wantedStates)
	}

	/** Heading is counter-clockwise positive.
	 *
	 * Chassis speeds have x represent forward, and y the left direction
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