package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class TimedAutoPlusSpin extends CommandBase {
    private DriveTrain driveTrain;
    private long timeEnd;
    private long timeToRun;
    private double vx;
    private double vy;
    private double vz;
    boolean isFinished;
    boolean flipPath;
    long currentTime;
    double gyroRotation;
    double wantedDegrees;
    double vzAdd;
    boolean spinBool;

    public TimedAutoPlusSpin(DriveTrain driveTrain, long timeToRun, double vx, double vy, double vz,
            double wantedDegrees) {
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
        spinBool = false;
    }

    @Override
    public void execute() {
        currentTime = System.currentTimeMillis();

        gyroRotation = driveTrain.getYaw();

        System.out.println("straightening");
        if (gyroRotation < wantedDegrees - 3) {
            vz = 1;
        } else if (gyroRotation > wantedDegrees + 3) {
            vz = -1;
        } else {
            vz = 0;
            spinBool = true;
        }

        if (currentTime < timeEnd) {
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vz, driveTrain.getGyroRotation(true)));
        } else if (currentTime >= timeEnd && !spinBool) {
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, vz, driveTrain.getGyroRotation(true)));
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
