package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ZeroHeading extends CommandBase {
    DriveTrain driveTrain;
    boolean done;

    public ZeroHeading(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        driveTrain.zeroHeading();
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
