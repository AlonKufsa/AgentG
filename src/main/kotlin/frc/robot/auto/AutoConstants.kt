package frc.robot.auto

import com.hamosad1657.lib.units.AngularVelocity
import com.pathplanner.lib.path.PathConstraints

object AutoConstants {
	const val MAX_VELOCITY_MPS = 3.0
	const val MAX_ACCEL_MPS_SQR = 3.0
	val MAX_ANGULAR_VELOCITY = AngularVelocity.fromRps(2.0)
	val MAX_ANGULAR_ACCELERATION = AngularVelocity.fromRps(4.0)

	val constraints = PathConstraints(
		MAX_VELOCITY_MPS,
		MAX_ACCEL_MPS_SQR,
		MAX_ANGULAR_VELOCITY.asRadPs,
		MAX_ANGULAR_ACCELERATION.asRadPs
	)
}