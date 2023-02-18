package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ZeroDrive extends CommandBase{
    private final DriveTrain driveTrain;

    public ZeroDrive(DriveTrain driveArg){
        this.driveTrain = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void execute() {
        ChassisSpeeds noMove = new ChassisSpeeds(0,0,0);
        driveTrain.drive(noMove);
        driveTrain.ZeroDrive();
    }
    
}
