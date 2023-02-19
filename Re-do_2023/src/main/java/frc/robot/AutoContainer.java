package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.ZeroDrive;
import frc.robot.Commands.ZeroVert;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
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

        return new SequentialCommandGroup(
                /*
                 * zero wheels, zero vert arm, drive forward + vert up, stop, open claw, (two
                 * ball query if true -> drive + spin + arm to pickup,
                 * stop, close claw), (three ball query if true -> vert up + drive + spin, stop,
                 * open claw, drive + spin + arm to pickup,
                 * stop, close claw), (balance query if true -> vert to hold, drive on station,
                 * balance, stop), (else vert to hold, drive away from midline, stop)
                 */

                (new ZeroDrive(driveTrain)
                        .alongWith(new ZeroVert(vertArm)))
                        .andThen((new ZeroDrive(driveTrain)) // should be an auto drive, not sure if needed
                                .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, vertHeight)))
                                .andThen(new StopDrive(driveTrain))

        );
    }

    public Trajectory makeTragectory(String trajectorystring) {
        String trajectortyJSOn = trajectorystring;
        Trajectory trajectory = new Trajectory();
        Path trajectoryPath = Filesystem.getOperatingDirectory().toPath().resolve(trajectortyJSOn);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable To open Trajectory" + trajectortyJSOn, e.getStackTrace());
        }
        return trajectory;
    }

    public Command getAutoDrive(String trajectoryString, Supplier<Pose2d> supplier){
        Trajectory trajectory = makeTragectory(trajectoryString);
        
        PIDController xController = new PIDController(0, 0, 0);
        PIDController yController = new PIDController(0, 0, 0);
       ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, new Constraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
       thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, supplier, null, new HolonomicDriveController(xController, yController, thetaController), null, driveTrain);
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> driveTrain.stopModules()));
        }
    }
