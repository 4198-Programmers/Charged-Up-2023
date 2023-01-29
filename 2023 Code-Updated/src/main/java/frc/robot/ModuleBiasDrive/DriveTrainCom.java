package frc.robot.ModuleBiasDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCom extends CommandBase {
    private final DriveTrainMod driveTrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier zSupplier;

    public DriveTrainCom(DriveTrainMod driveTrainArg, DoubleSupplier xValuesArg, DoubleSupplier yValuesArg,
            DoubleSupplier zValuesArg) {
        driveTrain = driveTrainArg;
        xSupplier = xValuesArg;
        ySupplier = yValuesArg;
        zSupplier = zValuesArg;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
                zSupplier.getAsDouble(), driveTrain.ManualGyroRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
