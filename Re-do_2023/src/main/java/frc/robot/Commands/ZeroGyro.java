package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.VertArm;

public class ZeroGyro extends CommandBase {
    DriveTrain drive;

    public ZeroGyro(DriveTrain driveArg) {
        this.drive = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void execute() {
        drive.zeroGyro();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return drive.getPitch() == 0;
    }

}
