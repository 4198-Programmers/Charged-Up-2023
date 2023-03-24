package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class DriveTrainCom extends CommandBase {
    private final DriveTrain drive;
    private final DoubleSupplier XSupplier;
    private final DoubleSupplier YSupplier;
    private final DoubleSupplier ZSupplier;
    private final boolean fieldOrientation;

    public DriveTrainCom(DriveTrain driveArg, DoubleSupplier XArg,
            DoubleSupplier YArg, DoubleSupplier ZArg, boolean fieldOrientation) {
        this.drive = driveArg;
        this.XSupplier = XArg;
        this.YSupplier = YArg;
        this.ZSupplier = ZArg;
        this.fieldOrientation = fieldOrientation;

        addRequirements(driveArg);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                XSupplier.getAsDouble(),
                YSupplier.getAsDouble(),
                ZSupplier.getAsDouble(),
                drive.getGyroRotation(fieldOrientation)));
        SmartDashboard.putBoolean("FieldOrientation", fieldOrientation);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
