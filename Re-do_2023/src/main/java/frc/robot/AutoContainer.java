package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AutoReach;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.SwerveDriveBase;
import frc.robot.Subsystems.VertArm;

//All auto is bot oriented
public class AutoContainer {
    private static int autoType;
    private static int locationChoose;
    private static int priorityLocal;

    public enum Location {

        Left(0),
        Middle(1),
        Right(2);

        private Location(int location) {
            locationChoose = location;
        }
    }

    public enum AutoType {

        OneElementNoBalance(0),
        TwoElementNoBalance(1),
        OneElementBalance(2),
        TwoElementBalance(3),
        ThreeElementBalance(4);

        private AutoType(int type) {
            autoType = type;
        }

    }

    public enum LevelPriority {

        Floor(0),
        Mid(1),
        Top(2);

        private LevelPriority(int priority) {
            priorityLocal = priority;
        }
    }

    public enum Autos{
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
        private Autos(String path){
            this.path = path;
        }
        public String getPath(){
            return path;
        }

    }

    static final SwerveDriveBase swerveDriveBase = new SwerveDriveBase();
    static final LazySusanSub lazySusanSub = new LazySusanSub();
    static final Pneumatics pneumatics = new Pneumatics();
    static final ReachArmSub reachArmSub = new ReachArmSub();
    static final VertArm vertArm = new VertArm();

    int[] locationVarOneArray = { 27, 84, 17 };
    // Will need multiple for different distance values, labelled when we know them
    int[] vertHeightArray = { 20, 40, 60 }; // Encoder values for very height based on level priority
    boolean[] twoElementQueryArray = { false, true, true, false, true };
    boolean[] threeElementQueryArray = { false, false, true, false, false };
    boolean[] balanceQueryArray = { false, false, true, true, true };
    /*
     * if the auto should get a second element,
     * should activate a command that starts by placing down the element on the
     * field, then go to get the next
     * Structuring arrays like this we will be able to use one method to run auto,
     * based on the different types,
     * this will allow not only to have different positions, but to use 'if'
     * statements to check how many elements we wish to manuever
     * Complicated but hopefully will improve ease of use, Arrays must be in order
     * to work [2-17]
     **/

    public SequentialCommandGroup autoRunCommand() {
        int varOne = locationVarOneArray[locationChoose];
        boolean twoElementQuery = twoElementQueryArray[autoType];
        boolean threeElementQuery = threeElementQueryArray[autoType];
        boolean balanceQuery = balanceQueryArray[autoType];
        int vertHeight = vertHeightArray[priorityLocal];
        // Following is just an example to understand the goal of this, with no
        // photonvision as I don't know if that works yet - [cp 2-17]

        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup getReadyToPutConOnGridCommand(){
        return new SequentialCommandGroup(new AutoVert( vertArm, 0.5, Constants.MAX_VERTICAL_POSITION).alongWith(
            new AutoReach(reachArmSub, () -> 0.5, 100).raceWith(new WaitCommand(1))),
            new TogglePneumatics(pneumatics, true),
            new AutoVert( vertArm, -0.5, Constants.MIN_VERTICAL_POSITION).alongWith(new ControlReach(reachArmSub, ()->-0.5, 100)).raceWith(new WaitCommand(1)));
    }
    public static SequentialCommandGroup putConeOnGridCommand(){
        return new SequentialCommandGroup();
    }
    public static SequentialCommandGroup getReadyToPickUpConeCommand(){
        return new SequentialCommandGroup();
    }
    public static SequentialCommandGroup pickUpConeCommand(){
        return new SequentialCommandGroup();
    }
    public static SequentialCommandGroup balanceCommand(){
        return new SequentialCommandGroup(
            new Balance(swerveDriveBase)
        );
    }
    public enum Actions{
        getReadyToPutConeOnGrid("GetReadyToPutConeOnGrid", getReadyToPutConOnGridCommand()),
        putConeOnGrid("PutConeOnGrid", putConeOnGridCommand()),
        getReadyToPickUpCone("GetReadyToPickUpCone", getReadyToPickUpConeCommand()),
        pickUpCone("PickUpCone", pickUpConeCommand()),
        balance("Balance", balanceCommand());

        private String action;
        private SequentialCommandGroup sequentialCommandGroup;
        private Actions(String action, SequentialCommandGroup sequentialCommandGroup){
            this.action = action;
            this.sequentialCommandGroup = sequentialCommandGroup;
        }
        public String getkey(){
            return action;
        }
        public SequentialCommandGroup getCommand(){
            return sequentialCommandGroup;
        }
    }
    HashMap<String, Command> eventMap = new HashMap<>();
    public void makeAutoCommand(Actions actions){
        eventMap.put(actions.getkey(), actions.getCommand());
    }

    public FollowPathWithEvents planPathExample(Autos autos, Actions finalAction){
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(autos.getPath(), new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
        makeAutoCommand(Actions.getReadyToPutConeOnGrid);
        Command finalCommand = finalAction.getCommand();
        FollowPathWithEvents command = new FollowPathWithEvents(finalCommand,  trajectory.getMarkers(), eventMap);
        return command;
    }    

}
