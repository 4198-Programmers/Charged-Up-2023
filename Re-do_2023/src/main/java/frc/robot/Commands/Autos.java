package frc.robot.Commands;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve;

public final class Autos {
    public static int locationChoice;
    public static int autoChoice;

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

    private Swerve swerve;
    private static SwerveAutoBuilder autoBuilder;
    private HashMap<String, Command> eventMap;
    SendableChooser<AutoTypes> autoChooser; 
    SendableChooser<Locations> locationChooser;

    private static Autos autos;

    public static Autos getInstance(){
        if(autos == null){
            autos = new Autos();
        }
        return autos;
    }



    public void autoInitialize(SendableChooser<AutoTypes> autoChooser, SendableChooser<Locations> locationChooser, HashMap<String, Command> eventMap, Swerve swerve){
        this.swerve = swerve;
        this.eventMap = eventMap;
        this.locationChooser = locationChooser;
        this.autoChooser = autoChooser;

        Autos.autoBuilder = new SwerveAutoBuilder(
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