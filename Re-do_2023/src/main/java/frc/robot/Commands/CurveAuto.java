package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class CurveAuto extends CommandBase {
    private DriveTrain driveTrain;
    private long timeEnd;
    private long timeToRun;
    private double vx;
    private double vy;
    boolean isFinished;
    boolean flipPath;
    long currentTime;
    double gyroRotation;

    public CurveAuto(DriveTrain driveTrain, long timeToRun, double vx, double vy) {
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

        if (currentTime < timeEnd) {
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0, driveTrain.getGyroRotation(true)));
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
