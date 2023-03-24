package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class CalibrateGyro extends CommandBase {
    DriveTrain drive;

    public CalibrateGyro(DriveTrain driveArg) {
        this.drive = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void execute() {
        drive.resetGyro();
    }


    @Override
    public boolean isFinished() {
        return drive.getYaw() == 0;
    }

}
