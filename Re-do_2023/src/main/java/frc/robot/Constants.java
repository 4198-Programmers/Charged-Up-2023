package frc.robot;

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
    public static final double ANGLE_KP = 0.02;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0.1;

//Angle Motor Constants
    public static final double ANGULAR_MAX_SPEED = 5880/60;
    public static final double ANGULAR_MAX_ACCELERATION = ANGULAR_MAX_SPEED / 60;

//Drive Motor Constants
    public static final double DRIVE_MAX_SPEED = 5880/60;
    public static final double DRIVE_MAX_ACCELERATION = DRIVE_MAX_SPEED / 60;

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

//Front Right Module Constants
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
    public static final int FRONT_RIGHT_CANCODER_ID = 11;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
    public static final double FRONT_RIGHT_X_FROM_CENTER = X_FROM_CENTER;
    public static final double FRONT_RIGHT_Y_FROM_CENTER = -Y_FROM_CENTER;
    public static final int FRONT_RIGHT_MODULE_NUMBER = 1;

//Back Left Module Constants
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
    public static final int BACK_LEFT_CANCODER_ID = 9;
    public static final double BACK_LEFT_ANGLE_OFFSET = 0;
    public static final double BACK_LEFT_X_FROM_CENTER = -X_FROM_CENTER;
    public static final double BACK_LEFT_Y_FROM_CENTER = Y_FROM_CENTER;
    public static final int BACK_LEFT_MODULE_NUMBER = 2;
    
//Back Right Module Constants
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 1;
    public static final int BACK_RIGHT_CANCODER_ID = 10;
    public static final double BACK_RIGHT_ANGLE_OFFSET = 0;
    public static final double BACK_RIGHT_X_FROM_CENTER = -X_FROM_CENTER;
    public static final double BACK_RIGHT_Y_FROM_CENTER = -Y_FROM_CENTER;
    public static final int BACK_RIGHT_MODULE_NUMBER = 3;
}