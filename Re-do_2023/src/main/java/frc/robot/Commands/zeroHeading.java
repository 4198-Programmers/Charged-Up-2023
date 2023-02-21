package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDriveBase;

public class ZeroHeading extends CommandBase{
    SwerveDriveBase swerveDriveBase;
    boolean done;

    public ZeroHeading(SwerveDriveBase swerveDriveBase){
        this.swerveDriveBase = swerveDriveBase;
        addRequirements(swerveDriveBase);
    }
    @Override
    public void initialize() {
        done = false;
    }
    @Override
    public void execute() {
        swerveDriveBase.zeroHeading();
        done = true;
    }
    @Override
    public boolean isFinished() {
        return done;
    }
}
