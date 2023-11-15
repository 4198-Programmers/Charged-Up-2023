package frc.robot.AttemptFour;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDrive extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSupplier, ySupplier, zSupplier;
    private Supplier<Boolean> fieldOrientedSupplier;

    public SwerveDrive(SwerveSubsystem swerveSubsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> zSupplier, Supplier<Boolean> fieldOrientedSupplier){
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        addRequirements(swerveSubsystem);
    }
    @Override
    public void execute() {
        swerveSubsystem.drive(xSupplier.get(), ySupplier.get(), zSupplier.get(), fieldOrientedSupplier.get());
    }
}
