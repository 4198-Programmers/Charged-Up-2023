package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class Balance extends CommandBase{
    private DriveTrain driveTrain;

    public Balance(DriveTrain driveArg){
        driveTrain = driveArg;
        addRequirements(driveTrain);
    }

    
    
}
