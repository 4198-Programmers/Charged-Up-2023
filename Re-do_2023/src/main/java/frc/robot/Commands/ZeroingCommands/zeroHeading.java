package frc.robot.Commands.ZeroingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;

public class zeroHeading extends CommandBase {
    Swerve driveTrain;
    boolean done;

    public zeroHeading(Swerve driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
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
