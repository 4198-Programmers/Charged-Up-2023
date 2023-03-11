package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveTrain;

public class Balance extends CommandBase {
    private DriveTrain driveTrain;
    float pitch;
    double speed;
    double pitchDouble;

    public Balance(DriveTrain driveArg) {
        driveTrain = driveArg;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        pitch = driveTrain.getPitch();
        speed = 0;
        pitchDouble = (double) pitch;

        if (pitchDouble < 45 || pitchDouble > -45) {
            speed = (pitchDouble / 45);
        } else if (pitchDouble > 45 || pitchDouble < -45) {
            speed = 1;
        } else {
            speed = 0;
        }
        driveTrain.drive(new ChassisSpeeds(speed, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
        // return driveTrain.BalanceDrive() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
