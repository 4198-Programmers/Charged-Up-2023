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
    private double vz;
    private ChassisSpeeds toSwerveSpeeds;
    boolean isFinished;
    boolean flipPath;
    long currentTime;
    double gyroRotation;
    double wantedDegrees;
    double vzAdd;

    public TimedAuto(DriveTrain driveTrain, long timeToRun, double vx, double vy, double vz, double wantedDegrees) {
        this.driveTrain = driveTrain;
        this.timeToRun = timeToRun;
        this.vx = vx;
        this.vy = vy;
        this.wantedDegrees = wantedDegrees;
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
        gyroRotation = driveTrain.getYaw();

        if (gyroRotation < wantedDegrees - 3) {
            vzAdd = 1;
        } else if (gyroRotation > wantedDegrees + 3) {
            vzAdd = -1;
        } else {
            vzAdd = 0;
        }

        if (currentTime < timeEnd) {
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vz + vzAdd, driveTrain.getGyroRotation(true)));
        } else {
            driveTrain.drive(new ChassisSpeeds(0, 0, 0));
            isFinished = true;
            System.out.println("Stop Auto");
        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
