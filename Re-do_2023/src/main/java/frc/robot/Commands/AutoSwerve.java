package frc.robot.Commands;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve;

public class AutoSwerve extends SequentialCommandGroup{
    public AutoSwerve(Swerve s_Swerve, String pathName){
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathName, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        HashMap<String, Command> eventMap = new HashMap<>();

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, 
        s_Swerve::resetOdometry, 
        Constants.Swerve.swerveKinematics,
        new PIDConstants(AutoConstants.kPXController, 0, 0), 
        new PIDConstants(AutoConstants.kPYController, 0, 0), 
        s_Swerve::setModuleStates, 
        eventMap, 
        s_Swerve);
            addCommands(
                autoBuilder.fullAuto(pathGroup));
    }
}
