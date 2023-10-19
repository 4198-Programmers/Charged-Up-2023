package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
//Make Swerve Modules
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;
//Make the Gyro
    private AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 100);
    
//Make the SwerveKinematics
    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        frontLeft.translationFromCenter(),
        frontRight.translationFromCenter(),
        backLeft.translationFromCenter(),
        backRight.translationFromCenter()
    );
    public SwerveDriveKinematics getKinematics(){
        return swerveKinematics;
    }
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    private SwerveModule[] swerveModules;
    private SwerveModulePosition[] swerveModulePositions;
    private SwerveDriveOdometry odometry;

    public SwerveSubsystem(){
        frontLeft = new SwerveModule(
        Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
        Constants.FRONT_LEFT_ANGLE_MOTOR_ID, 
        Constants.FRONT_LEFT_CANCODER_ID, 
        Constants.FRONT_LEFT_ANGLE_OFFSET, 
        Constants.FRONT_LEFT_X_FROM_CENTER, 
        Constants.FRONT_LEFT_Y_FROM_CENTER,
        Constants.FRONT_LEFT_MODULE_NUMBER);
        frontRight = new SwerveModule(
        Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
        Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
        Constants.FRONT_RIGHT_CANCODER_ID, 
        Constants.FRONT_RIGHT_ANGLE_OFFSET, 
        Constants.FRONT_RIGHT_X_FROM_CENTER, 
        Constants.FRONT_RIGHT_Y_FROM_CENTER,
        Constants.FRONT_RIGHT_MODULE_NUMBER);
        backLeft = new SwerveModule(
        Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
        Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
        Constants.BACK_LEFT_CANCODER_ID, 
        Constants.BACK_LEFT_ANGLE_OFFSET, 
        Constants.BACK_LEFT_X_FROM_CENTER, 
        Constants.BACK_LEFT_Y_FROM_CENTER,
        Constants.BACK_LEFT_MODULE_NUMBER);
        backRight = new SwerveModule(
        Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
        Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
        Constants.BACK_RIGHT_CANCODER_ID, 
        Constants.BACK_RIGHT_ANGLE_OFFSET, 
        Constants.BACK_RIGHT_X_FROM_CENTER, 
        Constants.BACK_RIGHT_Y_FROM_CENTER,
        Constants.BACK_RIGHT_MODULE_NUMBER);

        swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        swerveModulePositions = new SwerveModulePosition[]{
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        };
        odometry = new SwerveDriveOdometry(swerveKinematics, getGyroRotation(true), swerveModulePositions, new Pose2d(0, 0, new Rotation2d(0, 0)));
    }
//Gryo functions
    public float getYaw(){
        return gyro.getYaw();
    }
    public float getPitch(){
        return gyro.getPitch();
    }
    public float getXAcceleration(){
        return gyro.getWorldLinearAccelX();
    }
    public float getYAcceleration(){
        return gyro.getWorldLinearAccelY();
    }
    public boolean isMoving(){
        return gyro.isMoving();
    }
    public void zeroGyro(){
        gyro.zeroYaw();
    }
    public void resetGyro(){
        gyro.calibrate();
    }
    public boolean gyroConnected(){
        return gyro.isConnected();
    }
    public boolean calibratingGyro(){
        return gyro.isCalibrating();
    }

    public Rotation2d getGyroRotation(boolean fieldOriented){
        if(fieldOriented){
            return Rotation2d.fromDegrees(gyro.getYaw());
        }else{
            return Rotation2d.fromDegrees(0);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DRIVE_MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[Constants.FRONT_LEFT_MODULE_NUMBER]);
        frontRight.setDesiredState(desiredStates[Constants.FRONT_RIGHT_MODULE_NUMBER]);
        backLeft.setDesiredState(desiredStates[Constants.BACK_LEFT_MODULE_NUMBER]);
        backRight.setDesiredState(desiredStates[Constants.BACK_RIGHT_MODULE_NUMBER]);

    }
//Drive Function
    public void drive(double x, double y, double z, boolean fieldOriented){
        SlewRateLimiter xyLimiter = new SlewRateLimiter(Constants.DRIVE_MAX_ACCELERATION);
        SlewRateLimiter zLimiter = new SlewRateLimiter(Constants.ANGULAR_MAX_ACCELERATION);

        x = Math.abs(x) > Constants.DEADBAND ? x : 0.0;
        y = Math.abs(y) > Constants.DEADBAND ? y : 0.0;
        z = Math.abs(z) > Constants.DEADBAND ? z : 0.0;

        x = xyLimiter.calculate(x) * Constants.DRIVE_MAX_SPEED;
        y = xyLimiter.calculate(y) * Constants.DRIVE_MAX_SPEED;
        z = zLimiter.calculate(z) * Constants.ANGULAR_MAX_SPEED;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, getGyroRotation(fieldOriented));

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveKinematics.desaturateWheelSpeeds(states, Constants.DRIVE_MAX_SPEED);

        frontLeft.set(states[Constants.FRONT_LEFT_MODULE_NUMBER].speedMetersPerSecond, states[Constants.FRONT_LEFT_MODULE_NUMBER].angle.getDegrees());
        frontRight.set(states[Constants.FRONT_RIGHT_MODULE_NUMBER].speedMetersPerSecond, states[Constants.FRONT_RIGHT_MODULE_NUMBER].angle.getDegrees());
        backLeft.set(states[Constants.BACK_LEFT_MODULE_NUMBER].speedMetersPerSecond, states[Constants.BACK_LEFT_MODULE_NUMBER].angle.getDegrees());
        backRight.set(states[Constants.BACK_RIGHT_MODULE_NUMBER].speedMetersPerSecond, states[Constants.BACK_RIGHT_MODULE_NUMBER].angle.getDegrees());
    }
}
