package frc.robot.AttemptOne;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDrive extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier XSupplier, YSupplier, ZSupplier;
    private final boolean fieldOriented;

    public SwerveDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier ZSupplier, boolean fieldOriented){
        this.swerveSubsystem = swerveSubsystem;
        this.XSupplier = XSupplier;
        this.YSupplier = YSupplier;
        this.ZSupplier = ZSupplier;
        this.fieldOriented = fieldOriented;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        swerveSubsystem.drive(XSupplier.getAsDouble(), YSupplier.getAsDouble(), ZSupplier.getAsDouble(), fieldOriented);
    }
    @Override
    public void end(boolean interrupted){
        swerveSubsystem.drive(0, 0, 0, fieldOriented);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
