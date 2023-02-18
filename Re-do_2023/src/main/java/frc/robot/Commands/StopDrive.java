package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class StopDrive extends CommandBase{
    private final DriveTrain driveTrain;

    public StopDrive(DriveTrain driveArg){
        this.driveTrain = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void execute() {
        driveTrain.StopDrive();
    }
    
}
