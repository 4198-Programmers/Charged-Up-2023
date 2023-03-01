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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private Pneumatics pneumatics;
    private VertArm vertArm;
    private ReachArmSub reachArmSub;
    private LazySusanSub lazySusanSub;

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
        OneElementBalance(1),
        TwoElement(2),
        TwoElementBalance(3),
        ThreeElementBalance(4);

        private AutoTypes(int autoChoicePassIn){
            autoChoice = autoChoicePassIn;
        }

        public int getAutoType(){
            return autoChoice;
        }

    }
    public enum PlacementType{
        rightTop(0),
        rightMiddle(1),
        rightBottom(2),
        middleTop(3),
        middleMiddle(4),
        middleBottom(5),
        leftTop(6),
        leftMiddle(7),
        leftBottom(8);

        private int place;

        private PlacementType(int place){
            this.place = place;
        }
        public int getPlacement(){
            return place;
        }
    }

    private Swerve swerve;
    private static SwerveAutoBuilder autoBuilder;
    private HashMap<String, Command> eventMap;
    SendableChooser<AutoTypes> autoChooser; 
    SendableChooser<Locations> locationChooser;
    SendableChooser<PlacementType> placementChooser;

    private static AutoContainer autos;

    public static AutoContainer getInstance(){
        if(autos == null){
            autos = new AutoContainer();
        }
        return autos;
    }



    public void autoInitialize(SendableChooser<AutoTypes> autoChooser, SendableChooser<Locations> locationChooser, SendableChooser<PlacementType> placementChooser, HashMap<String, Command> eventMap, Swerve swerve, Pneumatics pneumatics, VertArm vertArm, ReachArmSub reachArmSub, LazySusanSub lazySusanSub){
        this.swerve = swerve;
        this.pneumatics = pneumatics;
        this.vertArm = vertArm;
        this.reachArmSub = reachArmSub;
        this.lazySusanSub = lazySusanSub;
        this.eventMap = eventMap;
        this.locationChooser = locationChooser;
        this.autoChooser = autoChooser;
        this.placementChooser = placementChooser;

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

            locationChooser.setDefaultOption("Right", Locations.Right);
            locationChooser.addOption("Middle", Locations.Middle);
            locationChooser.addOption("Left", Locations.Left);

            autoChooser.setDefaultOption("One Element", AutoTypes.OneElement);
            autoChooser.addOption("One Element Balance", AutoTypes.OneElementBalance);
            autoChooser.addOption("Two Elements", AutoTypes.TwoElement);
            autoChooser.addOption("Two Elements Balance", AutoTypes.TwoElementBalance);
            autoChooser.addOption("Three Elements Balance", AutoTypes.ThreeElementBalance);

            placementChooser.setDefaultOption("Top Right Place", PlacementType.rightTop);
            placementChooser.addOption("Middle Right Place", PlacementType.rightMiddle);
            placementChooser.addOption("Bottom Right Place", PlacementType.rightBottom);
            placementChooser.addOption("Top Middle Place", PlacementType.middleTop);
            placementChooser.addOption("Middle Middle Place", PlacementType.middleMiddle);
            placementChooser.addOption("Bottom Middle Place", PlacementType.middleBottom);
            placementChooser.addOption("Top Left Place", PlacementType.leftTop);
            placementChooser.addOption("Middle Left Place", PlacementType.leftMiddle);
            placementChooser.addOption("Bottom Left Place", PlacementType.leftBottom);

            if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.OneElement){
                autoBuilder.fullAuto(leftOneElementPath);
            }
            else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.OneElementBalance){
                autoBuilder.fullAuto(leftOneElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.TwoElement){
                autoBuilder.fullAuto(leftTwoElementPath);
            }
            else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.TwoElementBalance){
                autoBuilder.fullAuto(leftTwoElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.ThreeElementBalance){
                autoBuilder.fullAuto(leftThreeElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.OneElement){
                autoBuilder.fullAuto(middleOneElementPath);
            }
            else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.OneElementBalance){
                autoBuilder.fullAuto(middleOneElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.TwoElement){
                autoBuilder.fullAuto(middleTwoElementPath);
            }
            else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.TwoElementBalance){
                autoBuilder.fullAuto(middleTwoElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.ThreeElementBalance){
                autoBuilder.fullAuto(middleThreeElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.OneElement){
                autoBuilder.fullAuto(rightOneElementPath);
            }
            else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.OneElementBalance){
                autoBuilder.fullAuto(rightOneElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.TwoElement){
                autoBuilder.fullAuto(rightTwoElementPath);
            }
            else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.TwoElementBalance){
                autoBuilder.fullAuto(rightTwoElementBalancePath);
            }
            else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.ThreeElementBalance){
                autoBuilder.fullAuto(rightThreeElementBalancePath);
            }
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
        CommandBase auto;

        if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.OneElement){
            auto = autoBuilder.fullAuto(leftOneElementPath);
        }
        else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.OneElementBalance){
            auto = autoBuilder.fullAuto(leftOneElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.TwoElement){
            auto = autoBuilder.fullAuto(leftTwoElementPath);
        }
        else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.TwoElementBalance){
            auto = autoBuilder.fullAuto(leftTwoElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Left && autoChooser.getSelected() == AutoTypes.ThreeElementBalance){
            auto = autoBuilder.fullAuto(leftThreeElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.OneElement){
            auto = autoBuilder.fullAuto(middleOneElementPath);
        }
        else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.OneElementBalance){
            auto = autoBuilder.fullAuto(middleOneElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.TwoElement){
            auto = autoBuilder.fullAuto(middleTwoElementPath);
        }
        else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.TwoElementBalance){
            auto = autoBuilder.fullAuto(middleTwoElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Middle && autoChooser.getSelected() == AutoTypes.ThreeElementBalance){
            auto = autoBuilder.fullAuto(middleThreeElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.OneElement){
            auto = autoBuilder.fullAuto(rightOneElementPath);
        }
        else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.OneElementBalance){
            auto = autoBuilder.fullAuto(rightOneElementBalancePath);
        }
        else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.TwoElement){
            auto = autoBuilder.fullAuto(rightTwoElementPath);
        }
        else if(locationChooser.getSelected() == Locations.Right && autoChooser.getSelected() == AutoTypes.TwoElementBalance){
            auto = autoBuilder.fullAuto(rightTwoElementBalancePath);
        }
        else{
            auto = autoBuilder.fullAuto(rightThreeElementBalancePath);
        }
        return auto;
    }
    public CommandBase prepElementPlacement(){
        return new SequentialCommandGroup(eventMap.get("PrepElementPlacement"));
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

    public CommandBase prepElementCommand(){
        String event = new String();
        if(placementChooser.getSelected() == PlacementType.rightTop){
            event = "Right Top";
        }
        else if(placementChooser.getSelected() == PlacementType.rightMiddle){
            event = "Right Middle";
        }
        else if(placementChooser.getSelected() == PlacementType.rightBottom){
            event = "Right Bottom";
        }
        else if(placementChooser.getSelected() == PlacementType.leftTop){
            event = "Left Top";
        }
        else if(placementChooser.getSelected() == PlacementType.leftMiddle){
            event = "Left Middle";
        }
        else if(placementChooser.getSelected() == PlacementType.leftBottom){
            event = "Left Bottom";
        }
        else if(placementChooser.getSelected() == PlacementType.middleTop){
            event = "Middle Top";
        }
        else if(placementChooser.getSelected() == PlacementType.middleMiddle){
            event = "Middle Middle";
        }
        else if(placementChooser.getSelected() == PlacementType.middleBottom){
            event = "Middle Bottom";
        }
        return new PrintCommand(event);
    }
    public CommandBase placeElementCommand(){
        return new PrintCommand("Place Element");
        //return new SequentialCommandGroup(new TogglePneumatics(pneumatics, true));
    }
    public CommandBase prepElementPickupCommand(){
        return new PrintCommand("Prep Pickup Element");
        //return new SequentialCommandGroup(
    //   new InstantCommand(() -> vertArm.autoVert(0.5, Constants.VERT_PICKUP_POS), vertArm),
    //   new InstantCommand(() -> pneumatics.togglePneumatics(true), pneumatics)
    // ));
    }
    public CommandBase pickupElementCommand(){
        return new PrintCommand("Pickup Element");
        //return new SequentialCommandGroup(
    //   new InstantCommand(() -> pneumatics.togglePneumatics(true))
    // ));
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