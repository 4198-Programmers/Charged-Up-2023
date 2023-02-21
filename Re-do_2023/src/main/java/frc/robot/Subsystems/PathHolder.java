package frc.robot.Subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathHolder extends SubsystemBase {

    private static String path;

    public enum PathChoice {// Extra spacing for readability
        // Probably an easier way to integrate this for driving, this is for
        // functionality alone CP [2-21]

        Left_One_Element_No_Balance("LeftOneElementNoBalance"),
        Left_Two_Element_No_Balance("LeftTwoElementNoBalance"),
        Left_One_Element_Balance("LeftOneElementBalance"),
        Left_Two_Element_Balance("LeftTwoElementBalance"),
        Left_Three_Element_Balance("LeftThreeElementBalance"),

        Mid_One_Element_No_Balance("MidOneElementNoBalance"),
        Mid_Two_Element_No_Balance("MidTwoElementNoBalance"),
        Mid_One_Element_Balance("MidOneElementBalance"),
        Mid_Two_Element_Balance("MidTwoElementBalance"),
        Mid_Three_Element_Balance("MidThreeElementBalance"),

        Right_One_Element_No_Balance("RightOneElementNoBalance"),
        Right_Two_Element_No_Balance("RightTwoElementNoBalance"),
        Right_One_Element_Balance("RightOneElementBalance"),
        Right_Two_Element_Balance("RightTwoElementBalance"),
        Right_Three_Element_Balance("RightThreeElementBalance");

        private PathChoice(String chosenPath) {
            path = chosenPath;
        }
    }

    PathPlannerTrajectory examplePath = PathPlanner.loadPath(path, new PathConstraints(4, 3));
    PathPlannerState state;

    public double[] getPathVelocities(double matchTime) {
        state = (PathPlannerState) examplePath.sample(matchTime);
        double[] velocitiesAndRunTime = { state.velocityMetersPerSecond, 0, state.holonomicAngularVelocityRadPerSec,
                examplePath.getTotalTimeSeconds() };
        return velocitiesAndRunTime;
    }

}
