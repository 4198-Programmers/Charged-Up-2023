package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    //These give update the double values continuously
    private final Supplier<Double> xSupplier, ySupplier, zSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;


    public SwerveDriveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSupplier,
        Supplier<Double> ySupplier, Supplier<Double> zSupplier, Supplier<Boolean> fieldOrientedSupplier){
            this.swerveSubsystem = swerveSubsystem;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.zSupplier = zSupplier;
            this.fieldOrientedSupplier = fieldOrientedSupplier;
            this.xLimiter = new SlewRateLimiter(Constants.DRIVE_MAX_ACCELERATION_PER_SECOND);
            this.yLimiter = new SlewRateLimiter(Constants.DRIVE_MAX_ACCELERATION_PER_SECOND);
            this.zLimiter = new SlewRateLimiter(Constants.ANGULAR_MAX_ACCELERATION_PER_SECOND);

            addRequirements(swerveSubsystem);
        }
        @Override
        public void initialize() {
        }

        @Override
        public void execute() {
            //Gets real-time joystick inputs
            double xSpeed = xSupplier.get();
            double ySpeed = ySupplier.get();
            double zSpeed = zSupplier.get();

            //Applies a deadband
            xSpeed = Math.abs(xSpeed) > Constants.DEADBAND ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > Constants.DEADBAND ? ySpeed : 0.0;
            zSpeed = Math.abs(zSpeed) > Constants.DEADBAND ? zSpeed : 0.0;

            //Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * Constants.MAX_SPEED_METERS_PER_SECOND;
            ySpeed = yLimiter.calculate(ySpeed) * Constants.MAX_SPEED_METERS_PER_SECOND;
            zSpeed = zLimiter.calculate(zSpeed) * Constants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            //Construct the desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            if (fieldOrientedSupplier.get()){
                //Relative to Field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubsystem.getRotation2d());
            }else{
                //Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
            }
            //Convert chassis speeds to indivdual module states
            SwerveModuleState[] moduleStates = Constants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

            //Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        @Override
        public void end(boolean interrupted) {
            swerveSubsystem.stopModules();
        }
        @Override
        public boolean isFinished() {
            return false;
        }
}