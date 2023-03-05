package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class SlightTurnDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0.05);
    private long timeToRunMil = 500;
    private long timeRun;

    public SlightTurnDrive(DriveTrain driveArg) {
        this.driveTrain = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void initialize() {
        timeRun = System.currentTimeMillis();

    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - timeRun < timeToRunMil) {
            driveTrain.drive(speeds);
        } else {
            driveTrain.StopDrive();
        }
    }

}
