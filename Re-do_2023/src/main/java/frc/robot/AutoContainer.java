package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

//All auto is bot oriented
public class AutoContainer {

    /**
     * public enum Location {
     * 
     * Left(0),
     * Middle(1),
     * Right(2);
     * 
     * private Location(int location) {
     * locationChoose = location;
     * }
     * }
     * 
     * public enum AutoType {
     * 
     * OneElementNoBalance(0),
     * TwoElementNoBalance(1),
     * OneElementBalance(2),
     * TwoElementBalance(3),
     * ThreeElementBalance(4);
     * 
     * private AutoType(int type) {
     * autoType = type;
     * }
     * 
     * }
     * 
     * public enum LevelPriority {
     * 
     * Floor(0),
     * Mid(1),
     * Top(2);
     * 
     * private LevelPriority(int priority) {
     * priorityLocal = priority;
     * }
     * }
     */

    public enum Autos {
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
        RightThreeElementBalance("RightThreeElementBalance");

        private String path;

        private Autos(String path) {
            this.path = path;
        }

        public String getPath() {
            return path;
        }

    }

    // static DriveTrain driveTrain = new DriveTrain();
    // static LazySusanSub lazySusanSub = new LazySusanSub();
    // static Pneumatics pneumatics = new Pneumatics();
    // static ReachArmSub reachArmSub = new ReachArmSub();
    // static VertArm vertArm = new VertArm();


    /*
     * int[] locationVarOneArray = { 27, 84, 17 };
     * // Will need multiple for different distance values, labelled when we know
     * them
     * int[] vertHeightArray = { 20, 40, 60 }; // Encoder values for very height
     * based on level priority
     * boolean[] twoElementQueryArray = { false, true, true, false, true };
     * boolean[] threeElementQueryArray = { false, false, true, false, false };
     * boolean[] balanceQueryArray = { false, false, true, true, true };
     * 
     * if the auto should get a second element,
     * should activate a command that starts by placing down the element on the
     * field, then go to get the next
     * Structuring arrays like this we will be able to use one method to run auto,
     * based on the different types,
     * this will allow not only to have different positions, but to use 'if'
     * statements to check how many elements we wish to manuever
     * Complicated but hopefully will improve ease of use, Arrays must be in order
     * to work [2-17]
     * 
     * 
     * public SequentialCommandGroup autoRunCommand() {
     * // Again, leaving this just in case, but it probably won't be needed
     * int varOne = locationVarOneArray[locationChoose];
     * boolean twoElementQuery = twoElementQueryArray[autoType];
     * boolean threeElementQuery = threeElementQueryArray[autoType];
     * boolean balanceQuery = balanceQueryArray[autoType];
     * int vertHeight = vertHeightArray[priorityLocal];
     * // Following is just an example to understand the goal of this, with no
     * // photonvision as I don't know if that works yet - [cp 2-17]
     * 
     * return new SequentialCommandGroup(
     * 
     * // * zero wheels, zero vert arm, drive forward + vert up, stop, open claw,
     * (two
     * // * ball query if true -> drive + spin + arm to pickup,
     * // * stop, close claw), (three ball query if true -> vert up + drive + spin,
     * stop,
     * // * open claw, drive + spin + arm to pickup,
     * // * stop, close claw), (balance query if true -> vert to hold, drive on
     * station,
     * // * balance, stop), (else vert to hold, drive away from midline, stop)
     * 
     * 
     * (new ZeroDrive(driveTrain)
     * .alongWith(new ZeroVert(vertArm)))
     * .andThen((new ZeroDrive(driveTrain)) // should be an auto drive, not sure if
     * //needed
     * .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, vertHeight)))
     * .andThen(new StopDrive(driveTrain))
     * 
     * );
     * }
     * 
     */

    // public static SequentialCommandGroup PrepElementPlacement() {
    //     return new SequentialCommandGroup(new AutoVert(vertArm, 0.5, Constants.MAX_VERTICAL_POSITION).alongWith(
    //             new TogglePneumatics(pneumatics, true),
    //             new AutoVert(vertArm, -0.5, Constants.MIN_VERTICAL_POSITION)
    //                     .alongWith(new ControlReach(reachArmSub, () -> -0.5, 100)).raceWith(new WaitCommand(1))));
    // }

    public static SequentialCommandGroup PlaceElement() {
        return new SequentialCommandGroup(new PrintCommand("Place Element"));
    }

    public static SequentialCommandGroup PrepElementPickup() { // TODO Something exremely similar in
                                                               // PathHolder, just separate so fewer conflicts
        return new SequentialCommandGroup(new PrintCommand("Prep Element Pickup"));
    }

    public static SequentialCommandGroup PickupElement() {
        return new SequentialCommandGroup(new PrintCommand("PickupElement"));
    }

    public static SequentialCommandGroup Balance() {
        return new SequentialCommandGroup(new PrintCommand("Balance"));
    }

    public static SequentialCommandGroup InitializeAutoValues() {
        return new SequentialCommandGroup(new PrintCommand("Initialize Auto Values"));
    }

    // Start Emily Code - I don't want to try to read this I'm quick writing my own
    // stuff CP [2-21]

    public enum Actions {
        // prepElementPlacement("PrepElementPlacement", PrepElementPlacement()),
        placeElement("PlaceElement", PlaceElement()),
        prepElementPickup("PrepElementPickup", PrepElementPickup()),
        pickupElement("PickupElement", PickupElement()),
        balance("Balance", Balance()),
        initializeAutoValues("IntializeAutoValues", InitializeAutoValues());

        private String action;
        private SequentialCommandGroup sequentialCommandGroup;

        private Actions(String action, SequentialCommandGroup sequentialCommandGroup) {
            this.action = action;
            this.sequentialCommandGroup = sequentialCommandGroup;
        }

        public String getkey() {
            return action;
        }

        public SequentialCommandGroup getCommand() {
            return sequentialCommandGroup;
        }
    }

    HashMap<String, Command> eventMap = new HashMap<>();

    public void makeAutoCommand(Actions actions) {
        eventMap.put(actions.getkey(), actions.getCommand());
    }
    // Leaving this here just in case, but most likely will not be used CP [2-21]
    // public Trajectory makeTragectory(String trajectorystring) {
    // String trajectortyJSOn = trajectorystring;
    // Trajectory trajectory = new Trajectory();
    // Path trajectoryPath =
    // Filesystem.getOperatingDirectory().toPath().resolve(trajectortyJSOn);
    // try {
    // trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException e) {
    // DriverStation.reportError("Unable To open Trajectory" + trajectortyJSOn,
    // e.getStackTrace());
    // }
    // return trajectory;
    // }

    // public FollowPathWithEvents planPathExample(Autos autos, Command finalCommand) {
    //     PathPlannerTrajectory trajectory = PathPlanner.loadPath(autos.getPath(), new PathConstraints(
    //             Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
    //     HashMap<String, Command> eventMap = new HashMap<>();
    //     finalCommand = new Balance(driveTrain);
    //     eventMap.put("PutConeOnGrid",
    //             new SequentialCommandGroup(new AutoVert(vertArm, 0.5, Constants.MAX_VERTICAL_POSITION).alongWith(
    //                     new ControlReach(reachArmSub, () -> 0.5, 100).raceWith(new WaitCommand(1))),
    //                     new TogglePneumatics(pneumatics, true),
    //                     new AutoVert(vertArm, -0.5, Constants.MIN_VERTICAL_POSITION)
    //                             .alongWith(new ControlReach(reachArmSub, () -> -0.5, 100))
    //                             .raceWith(new WaitCommand(1))));
    //     FollowPathWithEvents command = new FollowPathWithEvents(finalCommand, trajectory.getMarkers(), eventMap);
    //     return command;
    // }

}
