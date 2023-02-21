package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveDriveBase;

public class TeleOpSwerve extends CommandBase{
    private final SwerveDriveBase swerveDriveBase;
    private final Supplier<Double> xSpeedSupplier, ySpeedSupplier, thetaSpeedSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    
    public TeleOpSwerve(SwerveDriveBase swerveDriveBase, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> thetaSupplier, Supplier<Boolean> fieldOrientedSupplier){
        this.swerveDriveBase = swerveDriveBase;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.thetaSpeedSupplier = thetaSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        addRequirements(swerveDriveBase);
    }
    @Override
    public void execute() {

        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double thetaSpeed = thetaSpeedSupplier.get();

        xSpeed =  MathUtil.applyDeadband(xSpeed, 0.001);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.001);
        thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.001);
        ChassisSpeeds chassisSpeeds;

        if(fieldOrientedSupplier.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, swerveDriveBase.getRotation2d());
        } else{
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        }

        SwerveModuleState[] moduleStates = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveDriveBase.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveBase.stopModules();
    }
}