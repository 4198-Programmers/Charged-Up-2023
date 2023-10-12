package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
//Creating all of the modules
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.FRONT_LEFT_MOTOR_ID, 
        Constants.FRONT_LEFT_ANGLE_ID, 
        Constants.FRONT_LEFT_DRIVE_ENCODER_REVERSED, 
        Constants.FRONT_LEFT_ANGLE_ENCODER_REVERSED, 
        Constants.FRONT_LEFT_ABSOLUTE_ENCODER_ID, 
        Constants.FRONT_LEFT_ENCODER_OFFSET, 
        Constants.FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.FRONT_RIGHT_MOTOR_ID, 
        Constants.FRONT_RIGHT_ANGLE_ID, 
        Constants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED, 
        Constants.FRONT_RIGHT_ANGLE_ENCODER_REVERSED, 
        Constants.FRONT_RIGHT_ABSOLUTE_ENCODER_ID, 
        Constants.FRONT_RIGHT_ENCODER_OFFSET, 
        Constants.FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule backLeft = new SwerveModule(
        Constants.BACK_LEFT_MOTOR_ID, 
        Constants.BACK_LEFT_ANGLE_ID, 
        Constants.BACK_LEFT_DRIVE_ENCODER_REVERSED, 
        Constants.BACK_LEFT_ANGLE_ENCODER_REVERSED, 
        Constants.BACK_LEFT_ABSOLUTE_ENCODER_ID, 
        Constants.BACK_LEFT_ENCODER_OFFSET, 
        Constants.BACK_LEFT_ABSOLUTE_ENCODER_REVERSED);

    private final SwerveModule backRight = new SwerveModule(
        Constants.BACK_RIGHT_MOTOR_ID, 
        Constants.BACK_RIGHT_ANGLE_ID, 
        Constants.BACK_RIGHT_DRIVE_ENCODER_REVERSED, 
        Constants.BACK_RIGHT_ANGLE_ENCODER_REVERSED, 
        Constants.BACK_RIGHT_ABSOLUTE_ENCODER_ID, 
        Constants.BACK_RIGHT_ENCODER_OFFSET, 
        Constants.BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED);
//Creating the gyro. This tells us which way is forward relative to the field, not robot
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem(){
        new Thread(() ->{
            try{
                Thread.sleep(1000);
            }catch (Exception e){}
        }).start();
    }
    //This resets the gyro and sets the front of the robot to be forward
    public void zeroHeading(){
        gyro.reset();
    }
    //Tells us where the robot is currently facing
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    //Creates a Rotation2d of the current angle
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }
    //Sets the speeds to zero
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    //Tells the robot to update the states
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}