package frc.robot.auto

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint

/**
 * Generates a path on they fly from waypoints and an end state
 */
fun generatePathFromWaypoints(
	pathPoints: List<PathPoint>,
	endState: GoalEndState,
	preventFlipping: Boolean,
): PathPlannerPath {
	val path = PathPlannerPath.fromPathPoints(
		pathPoints,
		AutoConstants.constraints,
		endState,
	)
	path.preventFlipping = preventFlipping
	return path
}