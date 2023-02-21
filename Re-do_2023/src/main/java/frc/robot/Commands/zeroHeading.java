package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class zeroHeading extends CommandBase {
    DriveTrain driveTrain;
    boolean done;

    public zeroHeading(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        driveTrain.zeroGyro();
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
