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
        double xSpeed =Math.abs( xSupplier.get()) > Constants.DEADBAND ? xSupplier.get() : 0;
        double ySpeed = Math.abs(ySupplier.get()) > Constants.DEADBAND ? ySupplier.get() : 0;
        double zSpeed = Math.abs(zSupplier.get()) > Constants.DEADBAND ? zSupplier.get() : 0;
        // System.out.println("XSpeed:" + xSpeed);
        // System.out.println("YSpeed" + ySpeed);
        // System.out.println("ZSpeed:" + zSpeed);
        swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, fieldOrientedSupplier.get());
    }
}
