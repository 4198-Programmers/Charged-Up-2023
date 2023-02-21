package frc.robot.Commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.AutoContainer.Autos;
import frc.robot.Subsystems.SwerveDriveBase;

public class AutoSwerve extends CommandBase{
    SwerveDriveBase swerveDriveBase;
    Autos autos;
    Trajectory trajectory;
    SwerveControllerCommand swerveControllerCommand;

    public AutoSwerve(SwerveDriveBase swerveDriveBase, Autos autos){
        this.swerveDriveBase = swerveDriveBase;
        this.autos = autos;
        addRequirements(swerveDriveBase);
    }
    @Override
    public void initialize() {
        trajectory = PathPlanner.loadPath(autos.getPath(), new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
        PIDController xController = new PIDController(Constants.X_PID_CONTROLLER_KP, 0, 0);
        PIDController yController = new PIDController(Constants.Y_PID_CONTROLLER_KP, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(Constants.THETA_PID_CONTROLLER_KP, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            swerveDriveBase::getPose, 
            Constants.SWERVE_DRIVE_KINEMATICS, 
            xController,
            yController, 
            thetaController, 
            swerveDriveBase::setModuleStates, 
            swerveDriveBase);
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(new InstantCommand(() -> swerveDriveBase.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveDriveBase.stopModules()));
    }
}
