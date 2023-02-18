package frc.robot;



import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

//All auto is bot oriented
public class AutoContainer {
    public static int autoType;
    public static int locationChoose;

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
        ThreeElementNoBalance(2),
        OneElementBalance(3),
        TwoElementBalance(4);


        private AutoType(int type) {
            autoType = type;
        }

    }

    DriveTrain driveTrain;
    LazySusanSub lazySusanSub;
    Pneumatics pneumatics;
    ReachArmSub reachArmSub;
    VertArm vertArm;

    public AutoContainer(DriveTrain driveTrain, LazySusanSub lazySusanSub, Pneumatics pneumatics,
            ReachArmSub reachArmSub, VertArm vertArm) {
        this.driveTrain = driveTrain;
        this.lazySusanSub = lazySusanSub;
        this.pneumatics = pneumatics;
        this.reachArmSub = reachArmSub;
        this.vertArm = vertArm;
    }

    SequentialCommandGroup[] locationGroup = { Left(), Middle(), Right() };
    SequentialCommandGroup[] typeCommand = { OneElement(), TwoElement(), ThreeElement(), OneElementBalance(), TwoElementBalance() };

    // public SequentialCommandGroup getAutoChoice(){

    // }

    public SequentialCommandGroup autoRunCommands(){
        SequentialCommandGroup locationCommand = locationGroup[locationChoose];
        SequentialCommandGroup typeCommandGroup = typeCommand[autoType];

        return new SequentialCommandGroup(locationCommand.andThen(typeCommandGroup));
    }

    public static SequentialCommandGroup Right() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup Left() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup Middle() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup OneElementBalance() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup TwoElementBalance() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup OneElement() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup TwoElement() {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup ThreeElement() {
        return new SequentialCommandGroup();
    }

    public void makeTragectory(){
        String trajectortyJSOn = "Desktop/PathWeaver/Paths/BlueLeftElementOneToBalance.wpilib.json";
        Trajectory trajectory = new Trajectory();
        Path trajectoryPath = Filesystem.getOperatingDirectory().toPath().resolve(trajectortyJSOn);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable To open Trajectory" + trajectortyJSOn, e.getStackTrace());
        }
    }

    }   