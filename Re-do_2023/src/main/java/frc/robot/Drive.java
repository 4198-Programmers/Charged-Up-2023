package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier zSupplier;
    private final boolean fieldOriented;

    public Drive(SwerveSubsystem swerveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                DoubleSupplier zSupplier, boolean fieldOriented){
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), zSupplier.getAsDouble(), swerveSubsystem.getRotation(fieldOriented)));
    }
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopSpeed();
    }
}