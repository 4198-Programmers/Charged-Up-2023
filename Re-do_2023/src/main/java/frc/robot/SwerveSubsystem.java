package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    private final SwerveModule[] swerveModules;

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

    @Override
    public void periodic() {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveKinematics.desaturateWheelSpeeds(states, Constants.DRIVE_MAX_SPEED);
    }
}
