package frc.robot.Subsystems;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.StopArm;
import frc.robot.Commands.TogglePneumatics;

public class PathHolder {
    private static VertArm vertArm;
    private static Pneumatics pneumatics;
    private static LazySusanSub lazySusan;
    private static String pathChosen;

    // public enum PathChoice {
    // LeftOneElement("LeftOneElement"),
    // LeftOneElementBalance("LeftOneElementBalance"),
    // LeftTwoElement("LeftTwoElement"),
    // LeftTwoElementBalance("LeftTwoElementBalance"),
    // LeftThreeElementBalance("LeftThreeElementBalance"),
    // MiddleOneElement("MiddleOneElement"),
    // MiddleOneElementBalance("MiddleOneElementBalance"),
    // MiddleTwoElement("MiddleTwoElement"),
    // MiddleTwoElementBalance("MiddleTwoElementBalance"),
    // MiddleThreeElementBalance("MiddleThreeElementBalance"),
    // RightOneElement("RightOneElement"),
    // RightOneElementBalance("RightOneElementBalance"),
    // RightTwoElement("RightTwoElement"),
    // RightTwoElementBalance("RightTwoElementBalance"),
    // RightThreeElementBalance("RightThreeElementBalance"),
    // DriveStraight("DriveStraight");

    // private PathChoice(String path) {
    // pathChosen = path;
    // }

    // }

    public void setPath(String path) {
        pathChosen = path;
    }

    public double[] getPathVelocities(double matchTime) {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(pathChosen, new PathConstraints(4, 3));
        PathPlannerState state = (PathPlannerState) examplePath.sample(matchTime);
        double[] velocitiesAndRunTime = { state.velocityMetersPerSecond, 0, state.holonomicAngularVelocityRadPerSec,
                examplePath.getTotalTimeSeconds() };

        return velocitiesAndRunTime;
    }

    public List<EventMarker> getPathMarks() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(pathChosen, new PathConstraints(4, 3));
        return examplePath.getMarkers();
    }

    public static Command PrepElementPlacementOne() {
        // return new SequentialCommandGroup(new PrintCommand("Prep Element
        // Placement"));
        // arm up to safety
        // spin to left degrees
        // arm to placement pos + increase nPlacement

        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new PrintCommand("In Between"))
                .andThen(new StopArm(vertArm))
                .andThen(new PrintCommand("In Between"))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                        Constants.LEFT_PLACEMENT_ENC_POS))
                .andThen(new PrintCommand("In Between"))
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_MID_SHELF_PLACEMENT_ENC_SIDES))
                .andThen(new PrintCommand("In Between"))
                .andThen(new StopArm(vertArm))
                .andThen(new PrintCommand("Done")));

    }

    public static Command PrepElementPlacementTwo() {
        // return new SequentialCommandGroup(new PrintCommand("Prep Element
        // Placement"));
        // arm up to safety
        // spin to left degrees
        // arm to placement pos + increase nPlacement

        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new PrintCommand("In Between"))
                .andThen(new StopArm(vertArm))
                .andThen(new PrintCommand("In Between"))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                        Constants.RIGHT_PLACEMENT_ENC_POS))
                .andThen(new PrintCommand("In Between"))
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_MID_SHELF_PLACEMENT_ENC_SIDES))
                .andThen(new PrintCommand("In Between"))
                .andThen(new StopArm(vertArm))
                .andThen(new PrintCommand("Done")));

    }

    public static Command PlaceElement() {
        // return new SequentialCommandGroup(new PrintCommand("Place Element"));
        // open claw
        return new SequentialCommandGroup(new PrintCommand("Place Element")
                .andThen(new TogglePneumatics(pneumatics, true)).andThen(new PrintCommand("Done")));
    }

    public static Command PrepElementPickup() { // TODO Something exremely similar in
                                                // PathHolder, just separate so fewer conflicts
        // return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup"));
        // arm up to safety
        // spin to 0
        // arm to pickup pos

        return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new PrintCommand("In Between"))
                .andThen(new StopArm(vertArm))
                .andThen(new PrintCommand("In Between"))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                        0))
                .andThen(new PrintCommand("In Between"))
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_PICKUP_POS))
                .andThen(new PrintCommand("In Between"))
                .andThen(new StopArm(vertArm))
                .andThen(new PrintCommand("Done")));
    }

    public static Command PickupElement() {
        // return new SequentialCommandGroup(new PrintCommand("PickupElement"));
        // close claw
        return new SequentialCommandGroup(new PrintCommand("Pickup Element")
                .andThen(new TogglePneumatics(pneumatics, false))
                .andThen(new PrintCommand("Done")));
    }

    public static Command Balance() {
        // return new SequentialCommandGroup(new PrintCommand("Balance"));
        // arm to safety
        // spin to 0
        // balance
        return new SequentialCommandGroup(new PrintCommand("Balance")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new PrintCommand("Done")));
    }

    public static Command InitializeAutoValues() {
        return new SequentialCommandGroup(new PrintCommand("Initialize Auto Values"));
        // reset element placement
        // reset element pickup
    }

    public static Command VertSafeSpin() {
        return (new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS));
    }

    public static Command SusanLeft() {
        return (new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, Constants.LEFT_PLACEMENT_ENC_POS));
    }

    public static Command VertSidesPlace() {
        return (new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_MID_SHELF_PLACEMENT_ENC_SIDES));
    }

    public static Command ZeroSusan() {
        return (new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0));
    }

    public static Command PickupPos() {
        return (new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_PICKUP_POS));
    }

    public HashMap<String, Command> getPathEventMap() {
        HashMap<String, Command> CPHashMap = new HashMap<>();
        CPHashMap.put("PrepElementPlacementOne", PrepElementPlacementOne());
        CPHashMap.put("PrepElementPlacementTwo", PrepElementPlacementTwo());
        CPHashMap.put("PlaceElement", PlaceElement());
        CPHashMap.put("PrepElementPickup", PrepElementPickup());
        CPHashMap.put("PickupElement", PickupElement());
        CPHashMap.put("Balance", Balance());
        CPHashMap.put("InitializeAutoValues", InitializeAutoValues());
        CPHashMap.put("VertSafeSpin", VertSafeSpin());
        CPHashMap.put("SusanLeft", SusanLeft());
        CPHashMap.put("VertSidesPlace", VertSidesPlace());
        CPHashMap.put("ZeroSusan", ZeroSusan());
        CPHashMap.put("PickupPos", PickupPos());

        return CPHashMap;
    }

}
