package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
//Joystick Ports
    public static final int PORT_ZERO = 0;
    public static final int PORT_ONE = 1;
    public static final int PORT_TWO = 2;

    public static final double DEADBAND = 0.1;

//Axis on the Joysticks
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int Z_AXIS = 2;
    public static final int SLIDER_AXIS = 3;

//Cancoder Inverted
    public static final boolean CANCODER_INVERTED = false;

//PIDController Constants
    public static final double ANGLE_KP = 0.6;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 12;
    public static final double ANGLE_FF = 0;

//Motor Constant
    public static final double MAX_NUMBER_OF_ROTATIONS = 5880;
//Angle Motor Constants
    public static final double ANGULAR_MAX_SPEED = 5880/60;
    public static final double ANGULAR_MAX_ACCELERATION = ANGULAR_MAX_SPEED / 60;
    public static final double ANGLE_GEAR_RATIO = 150/7;

    public static final boolean ANGLE_MOTOR_INVERTED = false;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = false;
    public static final double ANGLE_CONTINUOUS_CURRENT_LIMIT = 0;
    public static final double ANGLE_PEAK_CURRENT_LIMIT = 0;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0;

//Drive Motor Constants
    public static final double DRIVE_MAX_SPEED = 5880/60;
    public static final double DRIVE_MAX_ACCELERATION = DRIVE_MAX_SPEED / 60;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = 1;
    public static final boolean DRIVE_MOTOR_INVERTED = false;

//Robot Dimensions
    public static final double ROBOT_BASE_WIDTH_METERS = 0.62;
    public static final double ROBOT_BASE_LENGTH_METERS = 0.62;
    public static final double X_FROM_CENTER = ROBOT_BASE_LENGTH_METERS / 2;
    public static final double Y_FROM_CENTER = ROBOT_BASE_LENGTH_METERS / 2;

//Front Left Module Constants
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 7;
    public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 8;
    public static final int FRONT_LEFT_CANCODER_ID = 12;
    public static final double FRONT_LEFT_ANGLE_OFFSET = 0;
    public static final double FRONT_LEFT_X_FROM_CENTER = X_FROM_CENTER;
    public static final double FRONT_LEFT_Y_FROM_CENTER = Y_FROM_CENTER;
    public static final int FRONT_LEFT_MODULE_NUMBER = 0;
    // public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = new SwerveModuleConstants(
    //     FRONT_LEFT_DRIVE_MOTOR_ID, 
    //     FRONT_LEFT_ANGLE_MOTOR_ID, 
    //     FRONT_LEFT_CANCODER_ID, 
    //     FRONT_LEFT_ANGLE_OFFSET);

//Front Right Module Constants
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
    public static final int FRONT_RIGHT_CANCODER_ID = 11;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
    public static final double FRONT_RIGHT_X_FROM_CENTER = X_FROM_CENTER;
    public static final double FRONT_RIGHT_Y_FROM_CENTER = -Y_FROM_CENTER;
    public static final int FRONT_RIGHT_MODULE_NUMBER = 1;
    // public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = new SwerveModuleConstants(
    //     FRONT_RIGHT_DRIVE_MOTOR_ID, 
    //     FRONT_RIGHT_ANGLE_MOTOR_ID, 
    //     FRONT_RIGHT_CANCODER_ID, 
    //     FRONT_RIGHT_ANGLE_OFFSET);

//Back Left Module Constants
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
    public static final int BACK_LEFT_CANCODER_ID = 9;
    public static final double BACK_LEFT_ANGLE_OFFSET = 0;
    public static final double BACK_LEFT_X_FROM_CENTER = -X_FROM_CENTER;
    public static final double BACK_LEFT_Y_FROM_CENTER = Y_FROM_CENTER;
    public static final int BACK_LEFT_MODULE_NUMBER = 2;
    // public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = new SwerveModuleConstants(
    //     BACK_LEFT_DRIVE_MOTOR_ID, 
    //     BACK_LEFT_ANGLE_MOTOR_ID, 
    //     BACK_LEFT_CANCODER_ID, 
    //     BACK_LEFT_ANGLE_OFFSET);
    
//Back Right Module Constants
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 1;
    public static final int BACK_RIGHT_CANCODER_ID = 10;
    public static final double BACK_RIGHT_ANGLE_OFFSET = 0;
    public static final double BACK_RIGHT_X_FROM_CENTER = -X_FROM_CENTER;
    public static final double BACK_RIGHT_Y_FROM_CENTER = -Y_FROM_CENTER;
    public static final int BACK_RIGHT_MODULE_NUMBER = 3;
    // public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = new SwerveModuleConstants(
    //     BACK_RIGHT_DRIVE_MOTOR_ID, 
    //     BACK_RIGHT_ANGLE_MOTOR_ID, 
    //     BACK_RIGHT_CANCODER_ID, 
    //     BACK_RIGHT_ANGLE_OFFSET);
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(FRONT_LEFT_X_FROM_CENTER, FRONT_LEFT_Y_FROM_CENTER),
        new Translation2d(FRONT_RIGHT_X_FROM_CENTER, FRONT_RIGHT_Y_FROM_CENTER),
        new Translation2d(BACK_LEFT_X_FROM_CENTER, BACK_LEFT_Y_FROM_CENTER),
        new Translation2d(BACK_RIGHT_X_FROM_CENTER, BACK_RIGHT_Y_FROM_CENTER
        )
    );
    public static final boolean INVERTED_GYRO = false;
    public static final int DRIVE_WHEEL_REDUCTION = 0;
    public static final double DRIVE_WHEEL_DIAMETER = 0;
    public static final double ANGLE_REDUCTION = 0;
    public static final int NEO_ROUNDS_PER_MINUTE = 0;
}