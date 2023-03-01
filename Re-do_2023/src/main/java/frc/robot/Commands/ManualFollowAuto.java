package frc.robot.Commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ManualFollowAuto extends CommandBase{
    private DriveTrain driveTrain;
    private PathPlannerTrajectory path;
    private String pathName;
    private long timeStart;
    private double matchTime;
    private ChassisSpeeds toSwerveSpeeds;
    

    public ManualFollowAuto(DriveTrain driveTrain, String pathToFollow){
        this.driveTrain = driveTrain;
        pathName = pathToFollow;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        path = PathPlanner.loadPath(pathName, 4, 3);
        timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        matchTime = (double) ((System.currentTimeMillis() - timeStart) / 1000);
        PathPlannerState state = (PathPlannerState) path.sample(matchTime);

        if (matchTime <= path.getTotalTimeSeconds()) {
            toSwerveSpeeds = new ChassisSpeeds(state.velocityMetersPerSecond, 0, state.angularVelocityRadPerSec);
            driveTrain.drive(toSwerveSpeeds);
        } else {
            toSwerveSpeeds = new ChassisSpeeds(0, 0, 0);
            driveTrain.drive(toSwerveSpeeds);
            System.out.println("Stop Auto");
        }

    }
    
}
