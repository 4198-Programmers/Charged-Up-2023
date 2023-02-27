package frc.robot.Subsystems;

import java.util.ArrayList;
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
import frc.robot.Commands.RunPathAuto;
import frc.robot.Commands.TogglePneumatics;

public class PathHolder {
    private static VertArm vertArm;
    private static Pneumatics pneumatics;
    private ReachArmSub reachArm;
    private static LazySusanSub lazySusan;
    private static int nPlacement;
    private static String pathChosen;

    public enum PathChoice {
        LeftOneElement("LeftOneElement"),
        LeftOneElementBalance("LeftOneElementBalance"),
        LeftTwoElement("LeftTwoElement"),
        LeftTwoElementBalance("LeftTwoElementBalance"),
        LeftThreeElementBalance("LeftThreeElementBalance"),
        MiddleOneElement("MiddleOneElement"),
        MiddleOneElementBalance("MiddleOneElementBalance"),
        MiddleTwoElement("MiddleTwoElement"),
        MiddleTwoElementBalance("MiddleTwoElementBalance"),
        MiddleThreeElementBalance("MiddleThreeElementBalance"),
        RightOneElement("RightOneElement"),
        RightOneElementBalance("RightOneElementBalance"),
        RightTwoElement("RightTwoElement"),
        RightTwoElementBalance("RightTwoElementBalance"),
        RightThreeElementBalance("RightThreeElementBalance"), 
        DriveStraight("DriveStraight");

        private PathChoice(String path) {
            pathChosen = path;
        }

    }

    public PathHolder(VertArm vertArm, Pneumatics pneumatics, ReachArmSub reachArm,
            LazySusanSub lazySusan) {
        this.vertArm = vertArm;
        this.pneumatics = pneumatics;
        this.reachArm = reachArm;
        this.lazySusan = lazySusan;
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

    public static double placementPos(int numberPlaced) {
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

    public static SequentialCommandGroup PrepElementPlacement() {
        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement").andThen(new TogglePneumatics(pneumatics, !pneumatics.getChannel())));
        // arm up to safety
        // spin to left degrees
        // arm to placement pos + increase nPlacement

        // return new SequentialCommandGroup(new PrintCommand("Prep Element Placement")
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        // .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
        // placementPos(nPlacement)))
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC)));

    }

    public static SequentialCommandGroup PlaceElement() {
        return new SequentialCommandGroup(new PrintCommand("Place Element").andThen(new TogglePneumatics(pneumatics, !pneumatics.getChannel())));
        // open claw
        // return new SequentialCommandGroup(new PrintCommand("Place Element")
        // .andThen(new TogglePneumatics(pneumatics, true))
        // );
    }

    public static SequentialCommandGroup PrepElementPickup() { // TODO Something exremely similar in
                                                               // PathHolder, just separate so fewer conflicts
        return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup").andThen(new TogglePneumatics(pneumatics, !pneumatics.getChannel())));
        // arm up to safety
        // spin to 0
        // arm to pickup pos

        // return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup")
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        // .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
        // 0))
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_PICKUP_POS)));
    }

    public static SequentialCommandGroup PickupElement() {
        return new SequentialCommandGroup(new PrintCommand("PickupElement").andThen(new TogglePneumatics(pneumatics, !pneumatics.getChannel())));
        // close claw
        // return new SequentialCommandGroup(new PrintCommand("Pickup Element")
        // .andThen(new TogglePneumatics(pneumatics, false))
        // );
    }

    public static SequentialCommandGroup Balance() {
        return new SequentialCommandGroup(new PrintCommand("Balance").andThen(new TogglePneumatics(pneumatics, !pneumatics.getChannel())));
        // arm to safety
        // spin to 0
        // balance
        // return new SequentialCommandGroup(new PrintCommand("Balance")
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        // .andThen(new Balance));
    }

    public static SequentialCommandGroup InitializeAutoValues() {
        return new SequentialCommandGroup(new PrintCommand("Initialize Auto Values"));
        // reset element placement
        // reset element pickup
    }

    public HashMap<String, Command> getPathEventMap() {
        HashMap<String, Command> CPHashMap = new HashMap<>();
        CPHashMap.put("PrepElementPlacement", PrepElementPlacement());
        CPHashMap.put("PlaceElement", PlaceElement());
        CPHashMap.put("PrepElementPickup", PrepElementPickup());
        CPHashMap.put("PickupElement", PickupElement());
        CPHashMap.put("Balance", Balance());
        CPHashMap.put("InitializeAutoValues", InitializeAutoValues());

        return CPHashMap;
    }

}
