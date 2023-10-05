package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    //Axis
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int Z_AXIS = 2;
    public static final int SLIDER = 3;


    public static final double DRIVE_TRAIN_WIDTH = 0.62;
    public static final double DRIVE_TRAIN_LENGTH = 0.62;

    /**
     * +x - forward     <p>
     * -x - backwards   <p>
     * +y - left       <p>
     * -y - right
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        //Front Left
        new Translation2d(DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0),
        //Front Right
        new Translation2d(DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0),
        //Back Left
        new Translation2d(DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0),
        //Back Right
        new Translation2d(DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0)
    );
    
    // public static final double DriveEcoderRotationToMeter = 0;
    // public static final double DriveEncoderRPMToMeterPerSec = 0;
    // public static final double AngleEncoderRotationToRadian = 0;
    // public static final double AngleEncoderRPMToRadPerSec = 0;

    public static final double kp = 0;
    public static final double MAX_SPEED_METERS_PER_SECOND = 5880.0 / 60.0; //max of 5880 rotations per second
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = MAX_SPEED_METERS_PER_SECOND / Math.hypot(DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0);
    public static final double DRIVE_MAX_ACCELERATION_PER_SECOND = MAX_SPEED_METERS_PER_SECOND / 60;
    public static final double ANGULAR_MAX_ACCELERATION_PER_SECOND = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 60;
    public static final double DEADBAND = 0.1;

    //Front left module constants
    public static final int FRONT_LEFT_MOTOR_ID = 7;
    public static final int FRONT_LEFT_ANGLE_ID = 8;
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_LEFT_ANGLE_ENCODER_REVERSED = false;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_ID = 12;
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0;
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    //Front right module constants
    public static final int FRONT_RIGHT_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_ANGLE_ID = 4;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_ANGLE_ENCODER_REVERSED = false;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 11;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    //Back left module constants
    public static final int BACK_LEFT_MOTOR_ID = 5;
    public static final int BACK_LEFT_ANGLE_ID = 6;
    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_ANGLE_ENCODER_REVERSED = false;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_ID = 9;
    public static final double BACK_LEFT_ENCODER_OFFSET = 0;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    //Back right module constants
    public static final int BACK_RIGHT_MOTOR_ID = 2;
    public static final int BACK_RIGHT_ANGLE_ID = 1;
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_ANGLE_ENCODER_REVERSED = false;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_ID = 10;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;

}
