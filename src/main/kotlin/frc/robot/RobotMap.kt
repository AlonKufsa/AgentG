package frc.robot

object RobotMap {
	object SwerveMap {
		object FrontLeft {
			const val DRIVE_MOTOR_ID = 15
			const val STEER_MOTOR_ID = 5
			const val CAN_CODER_ID = 6
		}

		object FrontRight {
			const val DRIVE_MOTOR_ID = 16
			const val STEER_MOTOR_ID = 7
			const val CAN_CODER_ID = 8
		}

		object BackLeft {
			const val DRIVE_MOTOR_ID = 17
			const val STEER_MOTOR_ID = 9
			const val CAN_CODER_ID = 10
		}

		object BackRight {
			const val DRIVE_MOTOR_ID = 18
			const val STEER_MOTOR_ID = 11
			const val CAN_CODER_ID = 12
		}

		const val PIGEON_2_ID = 2
	}
}