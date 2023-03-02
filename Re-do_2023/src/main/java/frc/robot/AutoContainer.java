package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AutoReach;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.BalanceCommand;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.VertArm;

public final class AutoContainer {
    public static int locationChoice;
    public static int autoChoice;
    public static int placementChoice;
    public static int balanceChoice;
    public static double vertArmHeight;
    public static double susanHeading;
    public static double reachTime;

    private Pneumatics pneumatics;
    private VertArm vertArm;
    private ReachArmSub reachArmSub;
    private LazySusanSub lazySusanSub;
    private Swerve swerve;

    private static SwerveAutoBuilder autoBuilder;
    private HashMap<String, Command> eventMap;

    SendableChooser<Integer> autoChooser; 
    SendableChooser<Integer> locationChooser;
    SendableChooser<Integer> placementLevelChooser;
    SendableChooser<Integer> placementSideChooser;
    SendableChooser<Integer> balanceChooser;
    
    public CommandBase[][][] auto;
    public double[] placementSide;
    public double[] placementLevel;
    public PlacementSettings[][] placement;

    public enum Locations{
        Right(0),
        Middle(1),
        Left(2);

        private Locations(int locationChoicePassIn){
            locationChoice = locationChoicePassIn;
        }
        public int getLocation(){
            return locationChoice;
        }
    }

    public enum AutoTypes{
        OneElement(0),
        TwoElement(1),
        ThreeElement(2);

        private AutoTypes(int autoChoicePassIn){
            autoChoice = autoChoicePassIn;
        }

        public int getAutoType(){
            return autoChoice;
        }

    }
    public enum Balance{
        balance(1),
        noBalance(0);
        private Balance(int balancePassIn){
            balanceChoice = balancePassIn;
        }
        public int getBalanceChoice(){
            return balanceChoice;
        }

    }
    public enum PlacementLevel{
        bottom(0),
        middle(1),
        top(2);

        private int placeLevel;

        private PlacementLevel(int place){
            this.placeLevel = place;
        }
        public int getPlacement(){
            return placeLevel;
        }
    }

    public enum PlacementSide{
        right(0),
        middle(1),
        left(2);
        private int placeSide;
        
        private PlacementSide(int place){
            this.placeSide = place;
        }

        public int getPlacement(){
            return placeSide;
        }
    }

    public enum PlacementSettings{
        rightTop(Constants.SUSAN_RIGHT_HEADING, Constants.VERT_TOP_SHELF_PLACEMENT_ENC, Constants.REACH_TOP_TIME),
        rightMiddle(Constants.SUSAN_RIGHT_HEADING, Constants.VERT_MIDDLE_SHELF_PLACEMENT_ENC, Constants.REACH_MIDDLE_TIME),
        rightBottom(Constants.SUSAN_RIGHT_HEADING, Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC, Constants.REACH_BOTTOM_TIME),

        middleTop(Constants.SUSAN_MIDDLE_HEADING, Constants.VERT_TOP_SHELF_PLACEMENT_ENC, Constants.REACH_TOP_TIME),
        middleMiddle(Constants.SUSAN_MIDDLE_HEADING, Constants.VERT_MIDDLE_SHELF_PLACEMENT_ENC, Constants.REACH_MIDDLE_TIME),
        middleBottom(Constants.SUSAN_MIDDLE_HEADING, Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC, Constants.REACH_BOTTOM_TIME),

        leftTop(Constants.SUSAN_LEFT_HEADING, Constants.VERT_TOP_SHELF_PLACEMENT_ENC, Constants.REACH_TOP_TIME),
        leftMiddle(Constants.SUSAN_LEFT_HEADING, Constants.VERT_MIDDLE_SHELF_PLACEMENT_ENC, Constants.REACH_MIDDLE_TIME),
        leftBottom(Constants.SUSAN_LEFT_HEADING, Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC, Constants.REACH_BOTTOM_TIME);
        private PlacementSettings(double headingPassIn, double heightPassIn, double timePassIn){
            susanHeading = headingPassIn;
            vertArmHeight = heightPassIn;
            reachTime = timePassIn;
        }
        public double getHeading(){
            return susanHeading;
        }
        public double getHeight(){
            return vertArmHeight;
        }
        public double getTime(){
            return reachTime;
        }
    }
    
    private static AutoContainer autos;

    public static AutoContainer getInstance(){
        if(autos == null){
            autos = new AutoContainer();
        }
        return autos;
    }



    public void autoInitialize(SendableChooser<Integer> autoChooser, SendableChooser<Integer> locationChooser, SendableChooser<Integer> placementLevelChooser, SendableChooser<Integer> placementSideChooser, SendableChooser<Integer> balanceChooser, HashMap<String, Command> eventMap, Swerve swerve, Pneumatics pneumatics, VertArm vertArm, ReachArmSub reachArmSub, LazySusanSub lazySusanSub){
        this.swerve = swerve;
        this.pneumatics = pneumatics;
        this.vertArm = vertArm;
        this.reachArmSub = reachArmSub;
        this.lazySusanSub = lazySusanSub;
        this.eventMap = eventMap;
        this.locationChooser = locationChooser;
        this.autoChooser = autoChooser;
        this.placementLevelChooser = placementLevelChooser;
        this.placementSideChooser = placementSideChooser;
        this.balanceChooser = balanceChooser;

        auto = new CommandBase[3][3][2];
        placementSide = new double[3];
        placementLevel = new double[3];
        placement = new PlacementSettings[3][3];



        AutoContainer.autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, 
            swerve::resetOdometry, 
            Constants.Swerve.swerveKinematics, 
            new PIDConstants(AutoConstants.kPXController, 0, 0), 
            new PIDConstants(AutoConstants.kPThetaController, 0, 0), 
            swerve::setModuleStates, 
            eventMap,
            false,
            swerve);

            locationChooser.setDefaultOption("Right", Locations.Right.getLocation());
            locationChooser.addOption("Middle", Locations.Middle.getLocation());
            locationChooser.addOption("Left", Locations.Left.getLocation());

            autoChooser.setDefaultOption("One Element", AutoTypes.OneElement.getAutoType());
            autoChooser.addOption("Two Elements", AutoTypes.TwoElement.getAutoType());
            autoChooser.addOption("Three Elements", AutoTypes.ThreeElement.getAutoType());

            placementLevelChooser.setDefaultOption("Bottom Level", PlacementLevel.bottom.getPlacement());
            placementLevelChooser.addOption("Middle Level", PlacementLevel.middle.getPlacement());
            placementLevelChooser.addOption("Top Level", PlacementLevel.top.getPlacement());

            placementSideChooser.setDefaultOption("Left Location", PlacementSide.left.getPlacement());
            placementSideChooser.addOption("Middle Location", PlacementSide.middle.getPlacement());
            placementSideChooser.addOption("Right Location", PlacementSide.right.getPlacement());

            balanceChooser.setDefaultOption("No Balance", Balance.noBalance.getBalanceChoice());
            balanceChooser.addOption("Balance", Balance.balance.getBalanceChoice());

            //auto[location(0 = right, 1 = middle, 2 = left)][autoType(0 =  1Element, 1 = 2Element, 2 = 3Element)][Balance(1 = Balance, 0 = no Blance)]
            auto[0][0][0] = autoBuilder.fullAuto(rightOneElementPath);
            auto[0][0][1] = autoBuilder.fullAuto(rightOneElementBalancePath);
            auto[0][1][0] = autoBuilder.fullAuto(rightTwoElementPath);
            auto[0][1][1] = autoBuilder.fullAuto(rightTwoElementBalancePath);
            auto[0][2][1] = autoBuilder.fullAuto(rightThreeElementBalancePath);

            auto[1][0][0] = autoBuilder.fullAuto(middleOneElementPath);
            auto[1][0][1] = autoBuilder.fullAuto(middleOneElementBalancePath);
            auto[1][1][0] = autoBuilder.fullAuto(middleTwoElementPath);
            auto[1][1][1] = autoBuilder.fullAuto(middleTwoElementBalancePath);
            auto[1][2][1] = autoBuilder.fullAuto(middleThreeElementBalancePath);

            auto[2][0][0] = autoBuilder.fullAuto(leftOneElementPath);
            auto[2][0][1] = autoBuilder.fullAuto(leftOneElementBalancePath);
            auto[2][1][0] = autoBuilder.fullAuto(leftTwoElementPath);
            auto[2][1][1] = autoBuilder.fullAuto(leftTwoElementBalancePath);
            auto[2][2][1] = autoBuilder.fullAuto(leftThreeElementBalancePath);

            placement[0][0] = PlacementSettings.rightTop;
            placement[0][1] = PlacementSettings.rightMiddle;
            placement[0][2] = PlacementSettings.rightBottom;

            placement[1][0] = PlacementSettings.middleTop;
            placement[1][1] = PlacementSettings.middleMiddle;
            placement[1][2] = PlacementSettings.middleBottom;

            placement[2][0] = PlacementSettings.leftTop;
            placement[2][1] = PlacementSettings.leftMiddle;
            placement[2][2] = PlacementSettings.leftBottom;

            
    }

    // public CommandBase moveToTarget(Pose2d targetPose){
    //     Pose2d currentPose = swerve.getPose();
    //     Rotation2d heading = (targetPose.getTranslation().minus(currentPose.getTranslation())).getAngle();
    //     PathPlannerTrajectory trajectory = PathPlanner.generatePath(
    //         new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    //         new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()),
    //         new PathPoint(targetPose.getTranslation(), heading, targetPose.getRotation()));
    //     return new SequentialCommandGroup(
    //         autoBuilder.followPathWithEvents(trajectory),
    //         (new TeleopSwerve(swerve, () ->0, () -> 0, () -> 0, () -> false))
    //     );
    // }

    public CommandBase getAuto(){
        if(balanceChooser.getSelected() == Balance.balance.getBalanceChoice()){
            return new SequentialCommandGroup(auto[locationChooser.getSelected()][autoChooser.getSelected()][balanceChooser.getSelected()], new BalanceCommand(swerve));
        }else{
        return auto[locationChooser.getSelected()][autoChooser.getSelected()][balanceChooser.getSelected()];   
        }
    } 

    public CommandBase prepElementPlacement(){
        return new SequentialCommandGroup(eventMap.get("InitializeValues"), eventMap.get("PrepElementPlacement"));
    }
    public CommandBase placeElement(){
        return new SequentialCommandGroup(eventMap.get("PlaceElement"));
    }

    public CommandBase prepElementPickup(){
        return new SequentialCommandGroup(eventMap.get("PrepElementPickup"));
    }

    public CommandBase pickupElement(){
        return new SequentialCommandGroup(eventMap.get("PickupElement"));
    }

    public CommandBase initializeCommand(){
        return new SequentialCommandGroup(new InstantCommand(() -> swerve.zeroHeading()), new InstantCommand(() -> vertArm.ZeroArm()), new InstantCommand(() -> lazySusanSub.zeroPosition()));
    }

    public CommandBase prepElementPlacementCommand(){
        PlacementSettings settings = placement[placementSideChooser.getSelected()][placementLevelChooser.getSelected()];
        AutoSusan autoSusan = new AutoSusan(lazySusanSub, 0.3, settings.getHeading());
        AutoVert autoVert = new AutoVert(vertArm, 0.5, settings.getHeight());
        AutoReach autoReach = new AutoReach(reachArmSub, 0.5, settings.getTime());
        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement"), autoVert, autoSusan, autoReach);
    }
    public CommandBase placeElementCommand(){
        PlacementSettings settings = placement[placementSideChooser.getSelected()][placementLevelChooser.getSelected()];
        AutoVert autoVert = new AutoVert(vertArm, -0.5, settings.getTime());
        return new SequentialCommandGroup(new PrintCommand("Place Element"), new TogglePneumatics(pneumatics, true), autoVert);
    }
    public CommandBase prepElementPickupCommand(){
        //return new PrintCommand("Prep Pickup Element");
        return new SequentialCommandGroup(new PrintCommand("Prep Pickup Element"),
      new InstantCommand(() -> vertArm.autoVert(0.5, Constants.VERT_PICKUP_POS), vertArm),
      new InstantCommand(() -> pneumatics.togglePneumatics(true), pneumatics)
    );
    }
    public CommandBase pickupElementCommand(){
        // return new PrintCommand("Pickup Element");
        return new SequentialCommandGroup(new PrintCommand("Pickup Element"), 
      new InstantCommand(() -> pneumatics.togglePneumatics(true)),
      new InstantCommand(() -> vertArm.autoVert(0.5, Constants.VERT_SAFE_TO_SPIN_ENC_POS))
    );
    }

    static List<PathPlannerTrajectory> leftOneElementPath = PathPlanner.loadPathGroup("LeftOneElement", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> leftOneElementBalancePath = PathPlanner.loadPathGroup("LeftOneElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> leftTwoElementPath = PathPlanner.loadPathGroup("LeftTwoElement", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> leftTwoElementBalancePath = PathPlanner.loadPathGroup("LeftTwoElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> leftThreeElementBalancePath = PathPlanner.loadPathGroup("LeftThreeElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    static List<PathPlannerTrajectory> rightOneElementPath = PathPlanner.loadPathGroup("RightOneElement", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> rightOneElementBalancePath = PathPlanner.loadPathGroup("RightOneElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> rightTwoElementPath = PathPlanner.loadPathGroup("RightTwoElement", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> rightTwoElementBalancePath = PathPlanner.loadPathGroup("RightTwoElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> rightThreeElementBalancePath = PathPlanner.loadPathGroup("RightThreeElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    static List<PathPlannerTrajectory> middleOneElementPath = PathPlanner.loadPathGroup("MiddleOneElement", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> middleOneElementBalancePath = PathPlanner.loadPathGroup("MiddleOneElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> middleTwoElementPath = PathPlanner.loadPathGroup("MiddleTwoElement", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> middleTwoElementBalancePath = PathPlanner.loadPathGroup("MiddleTwoElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    static List<PathPlannerTrajectory> middleThreeElementBalancePath = PathPlanner.loadPathGroup("MiddleThreeElementBalance", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

}