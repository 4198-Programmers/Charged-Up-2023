package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ConditionalLock extends CommandBase{
    DriveTrain drive;

    public ConditionalLock(DriveTrain drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if(drive.isMoving()) {

        } else {
            drive.drive(new ChassisSpeeds(0, 0, 0.01));
        }
    }
}
