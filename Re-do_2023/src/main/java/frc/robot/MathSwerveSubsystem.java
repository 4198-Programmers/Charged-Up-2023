package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MathSwerveSubsystem extends SubsystemBase{

    private AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 100);
    MathSwerveModule frontLeft;
    MathSwerveModule frontRight;
    MathSwerveModule backLeft;
    MathSwerveModule backRight;
    public MathSwerveSubsystem(){
        frontLeft = new MathSwerveModule(
            Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
            Constants.FRONT_LEFT_ANGLE_MOTOR_ID, 
            Constants.FRONT_LEFT_CANCODER_ID, 
            Constants.FRONT_LEFT_ANGLE_OFFSET);
        frontRight = new MathSwerveModule(
            Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
            Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
            Constants.FRONT_RIGHT_CANCODER_ID, 
            Constants.FRONT_RIGHT_ANGLE_OFFSET);
        backLeft = new MathSwerveModule(
            Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
            Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
            Constants.BACK_LEFT_CANCODER_ID, 
            Constants.BACK_LEFT_ANGLE_OFFSET);
        backRight = new MathSwerveModule(
            Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
            Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
            Constants.BACK_RIGHT_CANCODER_ID, 
            Constants.BACK_RIGHT_ANGLE_OFFSET);
    }
        /**
     * The return depends on if the robot is field oriented  or robot oriented<p>
     * If it is field oriented, the return is based on the yaw of the gyro.<p>
     * If it is robot oriented, the return will always have the fron of the robot be forward.<p>
     * @param fieldOriented This is a boolean value of whether or not the robot is field oriented or not.
     * @return the Rotation2d of the robot.
     */
    public Rotation2d getGyroRotation(boolean fieldOriented){
        if(fieldOriented){
            return Rotation2d.fromDegrees(gyro.getYaw());
        }else{
            return Rotation2d.fromDegrees(0);
        }
    }

    public void drive(double FWD, double STR, double RCW, boolean fieldOriented){
        double temp = FWD * Math.cos(getGyroRotation(fieldOriented).getDegrees()) 
            + STR * Math.sin(getGyroRotation(fieldOriented).getDegrees());
        STR = -FWD * Math.sin(getGyroRotation(fieldOriented).getDegrees()) 
            + STR * Math.cos(getGyroRotation(fieldOriented).getDegrees());
        FWD = temp;
        double L = Constants.ROBOT_BASE_LENGTH_METERS;
        double W = Constants.ROBOT_BASE_WIDTH_METERS;
        double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));
        double A = STR - RCW * (L/R);
        double B = STR + RCW * (L/R);
        double C = FWD - RCW * (W/R);
        double D = FWD + RCW * (W/R);

        frontLeft.setDriveSpeed(B, D);
        frontRight.setDriveSpeed(B, C);
        backLeft.setDriveSpeed(A, D);
        backRight.setDriveSpeed(A, C);

        frontLeft.setAngleSpeed(B, D);
        frontRight.setAngleSpeed(B, C);
        backLeft.setAngleSpeed(A, D);
        backRight.setAngleSpeed(A, C);

    }
}
