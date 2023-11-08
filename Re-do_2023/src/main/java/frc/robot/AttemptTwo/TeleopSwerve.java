package frc.robot.AttemptTwo;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TeleopSwerve extends CommandBase{
    
    private double rotation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Swerve swerveSubsystem;
    private DoubleSupplier xSupplier, ySupplier, zSupplier;
    
    public TeleopSwerve(Swerve swerveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldRelative, boolean openLoop){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double ySpeed = ySupplier.getAsDouble();
        double xSpeed = xSupplier.getAsDouble();
        double zSpeed = zSupplier.getAsDouble();

        ySpeed = (Math.abs(ySpeed)) < Constants.DEADBAND ? 0 : ySpeed;
        xSpeed = (Math.abs(xSpeed)) < Constants.DEADBAND ? 0 : xSpeed;
        zSpeed = (Math.abs(zSpeed)) < Constants.DEADBAND ? 0 : zSpeed;

        rotation = zSpeed * Constants.ANGULAR_MAX_SPEED;
        swerveSubsystem.drive(xSpeed, ySpeed, rotation, fieldRelative, openLoop);
    }
}
