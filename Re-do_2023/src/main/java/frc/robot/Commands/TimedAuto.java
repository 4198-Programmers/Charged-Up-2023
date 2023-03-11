package frc.robot.Commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class TimedAuto extends CommandBase {
    private DriveTrain driveTrain;
    private long timeEnd;
    private double matchTime;
    private long timeToRun;
    private double vx;
    private double vy;
    private ChassisSpeeds toSwerveSpeeds;
    boolean isFinished;
    boolean flipPath;
    long currentTime;

    public TimedAuto(DriveTrain driveTrain, long timeToRun, double vx, double vy) {
        this.driveTrain = driveTrain;
        this.timeToRun = timeToRun;
        this.vx = vx;
        this.vy = vy;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        timeEnd = System.currentTimeMillis() + timeToRun;
        isFinished = false;
    }

    @Override
    public void execute() {
        currentTime = System.currentTimeMillis();

        if ( currentTime < timeEnd) {
            toSwerveSpeeds = new ChassisSpeeds(vx, vy, 0);
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
