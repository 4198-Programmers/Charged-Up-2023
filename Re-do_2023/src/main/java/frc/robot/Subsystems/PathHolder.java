package frc.robot.Subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathHolder extends SubsystemBase {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("LeftOneElement", new PathConstraints(1, 0.5));
    PathPlannerState state;

    public double[] getPathVelocities(double matchTime) {
        state = (PathPlannerState) examplePath.sample(matchTime);
        double[] velocities = { state.velocityMetersPerSecond, 0, state.angularVelocityRadPerSec };
        return velocities;
    }

}
