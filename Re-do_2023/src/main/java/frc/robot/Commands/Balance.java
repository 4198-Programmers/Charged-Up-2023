package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.DriveTrain;

public class Balance extends CommandBase {
    private DriveTrain driveTrain;
    float pitch;
    double speed;
    double pitchDouble;
    boolean done;
    double pitchAngleDegrees;
    boolean isFinished;
    long timeEnded;
    WaitCommand wait = new WaitCommand(0.5);

    public Balance(DriveTrain driveArg) {
        driveTrain = driveArg;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        isFinished = false;
        done = false;
    }

    @Override
    public void execute() {
        pitch = driveTrain.getPitch();
        // speed = 0.5;
        pitchDouble = (double) pitch;

        // if (pitchDouble > 5) {
        // speed = 0.5;
        // wait.execute();
        // } else if (pitchDouble > 2 && pitchDouble < 5) {
        // speed = 0.15;
        // wait.execute();
        // } else if (pitchDouble < -2 && pitchDouble > -5) {
        // speed = -0.15;
        // wait.execute();
        // } else if (pitchDouble < -5) {
        // speed = -0.5;
        // wait.execute();
        // } else {
        // speed = 0;
        // }
        pitchAngleDegrees = (driveTrain.getPitch() - 3.8); // because the range is -5 - 16 when it needs to be -x -> x

        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        speed = Math.sin(pitchAngleRadians) * 2;

        if (Math.abs(pitchAngleDegrees) > 2 || driveTrain.getXAccel() > 0.01) {

            driveTrain.drive(new ChassisSpeeds(speed, 0, 0));

        } else {
            driveTrain.drive(new ChassisSpeeds(0, 0, 0.1));
            timeEnded = System.currentTimeMillis();
        }

        System.out.println("Accel X" + driveTrain.getXAccel());

        // System.out.println("Pitch: " + pitch);
        System.out.println("Pitch: " + pitchAngleDegrees);

    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pitchAngleDegrees) < 2 && Math.abs(driveTrain.getXAccel()) < 0.01
                && System.currentTimeMillis() > timeEnded + 500);
        // return driveTrain.BalanceDrive() == 0;
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0, 0, 0.1));
        driveTrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
