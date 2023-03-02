package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ZeroDrive extends CommandBase {
    private final DriveTrain driveTrain;

    public ZeroDrive(DriveTrain driveArg) {
        this.driveTrain = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void execute() {
        driveTrain.StopDrive();
        driveTrain.ZeroDrive();
    }

}
