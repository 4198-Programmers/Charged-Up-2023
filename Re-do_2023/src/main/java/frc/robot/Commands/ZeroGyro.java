package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ZeroGyro extends CommandBase {
    DriveTrain drive;
    boolean done;

    public ZeroGyro(DriveTrain driveArg) {
        this.drive = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        drive.zeroGyro();
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
