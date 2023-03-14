package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class AutoDriveBalance extends CommandBase {
    private final DriveTrain drive;
    private final double XSupplier;
    private final double YSupplier;
    private final double ZSupplier;
    boolean isFinished;
    double starngPitch;

    public AutoDriveBalance(DriveTrain driveArg, double XArg,
            double YArg, double ZArg) {
        drive = driveArg;
        XSupplier = XArg;
        YSupplier = YArg;
        ZSupplier = ZArg;
        addRequirements(driveArg);
    }

    @Override
    public void initialize() {
        starngPitch = drive.getPitch();
        isFinished = false;
    }

    @Override
    public void execute() {
        if(starngPitch > 10) {
            drive.drive(new ChassisSpeeds(
                    XSupplier,
                    YSupplier,
                    ZSupplier));
            } else {
                drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
                isFinished = true;
            }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
