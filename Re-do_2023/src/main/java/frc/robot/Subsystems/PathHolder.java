package frc.robot.Subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.TogglePneumatics;

public class PathHolder extends SubsystemBase {
    private static String path;
    private  VertArm vertArm;
    private  Pneumatics pneumatics;
    private  ReachArmSub reachArm;
    private  LazySusanSub lazySusan;
    private  int nPlacement;
    private  Swerve swerve;

    public PathHolder(VertArm vertArm, Pneumatics pneumatics, ReachArmSub reachArm,
            LazySusanSub lazySusan, Swerve swerve) {
        this.vertArm = vertArm;
        this.pneumatics = pneumatics;
        this.reachArm = reachArm;
        this.lazySusan = lazySusan;
        this.swerve = swerve;
    }

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

    private double[] getPathVelocities(double matchTime) {
        state = (PathPlannerState) examplePath.sample(matchTime);
        double[] velocitiesAndRunTime = { state.velocityMetersPerSecond, 0, state.holonomicAngularVelocityRadPerSec,
                examplePath.getTotalTimeSeconds() };

        return velocitiesAndRunTime;
    }

    private double placementPos(int numberPlaced) {
        if (numberPlaced == 0) {
            nPlacement++;
            System.out.println("Left Placement");
            return Constants.LEFT_PLACEMENT_DEGREES;
        } else if (numberPlaced == 1) {
            nPlacement++;
            System.out.println("Right Placement");
            return Constants.MID_PLACEMENT_DEGREES;
        } else {
            System.out.println("No Placement");
            return 0;
        }
    }

    private  SequentialCommandGroup PrepElementPlacement() {
    //return new SequentialCommandGroup(new PrintCommand("Prep Element Placement"));
        // arm up to safety
        // spin to left degrees
        // arm to placement pos + increase nPlacement

        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement")
        .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
        placementPos(nPlacement)))
        .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC)));

    }

    private SequentialCommandGroup PlaceElement() {
        //return new SequentialCommandGroup(new PrintCommand("Place Element"));
        // open claw
        return new SequentialCommandGroup(new PrintCommand("Place Element")
        .andThen(new TogglePneumatics(pneumatics, true))
        );
    }

    private SequentialCommandGroup PrepElementPickup() { // TODO Something exremely similar in
                                                               // PathHolder, just separate so fewer conflicts
        //return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup"));
        // arm up to safety
        // spin to 0
        // arm to pickup pos

        return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup")
        .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
        0))
        .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        Constants.VERT_PICKUP_POS)));
    }

    private  SequentialCommandGroup PickupElement() {
        //return new SequentialCommandGroup(new PrintCommand("PickupElement"));
        // close claw
        return new SequentialCommandGroup(new PrintCommand("Pickup Element")
        .andThen(new TogglePneumatics(pneumatics, false))
        );
    }

    private  SequentialCommandGroup Balance() {
        //return new SequentialCommandGroup(new PrintCommand("Balance"));
        // arm to safety
        // spin to 0
        // balance
        return new SequentialCommandGroup(new PrintCommand("Balance")
        .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        Constants.VERT_SAFE_TO_SPIN_ENC_POS)));
        //.andThen(new ));
    }

    private  SequentialCommandGroup InitializeAutoValues() {
        return new SequentialCommandGroup(new PrintCommand("Initialize Auto Values"));
        // reset element placement
        // reset element pickup
    }

    private List<EventMarker> getPathMarks() {
        return examplePath.getMarkers();
    }

    private HashMap<String, Command> getPathEventMap() {
        HashMap<String, Command> CPHashMap = new HashMap<>();
        CPHashMap.put("PrepElementPlacement", PrepElementPlacement());
        CPHashMap.put("PlaceElement", PlaceElement());
        CPHashMap.put("PrepElementPickup", PrepElementPickup());
        CPHashMap.put("PickupElement", PickupElement());
        CPHashMap.put("Balance", Balance());
        CPHashMap.put("InitializeAutoValues", InitializeAutoValues());
        return CPHashMap;
    }

    public Command createAutoPath(){
       return new FollowPathWithEvents(swerve.swerveTrajectory(examplePath), getPathMarks(), getPathEventMap());
    }

}