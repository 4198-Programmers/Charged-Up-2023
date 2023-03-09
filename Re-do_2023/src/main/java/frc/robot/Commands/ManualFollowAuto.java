package frc.robot.Commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ManualFollowAuto extends CommandBase {
    private DriveTrain driveTrain;
    private PathPlannerTrajectory path;
    private String pathName;
    private long timeStart;
    private double matchTime;
    private ChassisSpeeds toSwerveSpeeds;
    boolean isFinished;
    boolean flipPath;

    public ManualFollowAuto(DriveTrain driveTrain, String pathToFollow, boolean flipPath) {
        this.driveTrain = driveTrain;
        pathName = pathToFollow;
        this.flipPath = flipPath;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        path = PathPlanner.loadPath(pathName, 4, 3);
        timeStart = System.currentTimeMillis();
        isFinished = false;
    }

    @Override
    public void execute() {
        matchTime = (double) ((System.currentTimeMillis() - timeStart) / 1000);
        PathPlannerState state = (PathPlannerState) path.sample(matchTime);
        double driveMod;
        if (flipPath) {
            driveMod = -1;
        } else if (!flipPath) {
            driveMod = 1;
        } else {
            driveMod = 0;
        }

        if (matchTime <= path.getTotalTimeSeconds()) {
            toSwerveSpeeds = new ChassisSpeeds(state.velocityMetersPerSecond * driveMod, -state.angularVelocityRadPerSec * driveMod, //state.velocityMetersPerSecond * driveMod
            0);
            System.out.println(state.angularVelocityRadPerSec -0.071 + "angular");
            System.out.println(state.velocityMetersPerSecond + "drive");
            // Auto way too fast
            driveTrain.drive(toSwerveSpeeds);
        } else {
            toSwerveSpeeds = new ChassisSpeeds(0, 0, 0);
            driveTrain.drive(toSwerveSpeeds);
            isFinished = true;
            System.out.println("Stop Auto");
        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
