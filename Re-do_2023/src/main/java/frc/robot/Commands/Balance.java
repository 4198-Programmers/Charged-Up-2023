package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class Balance extends CommandBase {
    private DriveTrain driveTrain;

    public Balance(DriveTrain driveArg) {
        driveTrain = driveArg;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.drive(new ChassisSpeeds(driveTrain.BalanceDrive(), 0, 0));
    }

    @Override
    public boolean isFinished() {
        return driveTrain.BalanceDrive() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
