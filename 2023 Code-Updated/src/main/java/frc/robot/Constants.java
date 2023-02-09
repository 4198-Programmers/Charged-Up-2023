package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

//Arm Motors
    public static final byte SHOULDER_MOTOR = 0;
    public static final byte ELBOW_MOTOR = 0;
    public static final byte HAND_MOTOR = 0;
    public static final byte RETRACTABLE_ARM_MOTOR = 0;

    public static final byte LEFT_JOYSTICK = 0;

    //apriltag variables
    public static final double WANTED_DISTANCE_LEFT = 0;
    public static final double WANTED_DISTANCE_MID = 48;
    public static final double WANTED_DISTANCE_RIGHT = 0;
    public static final double WANTED_YAW_LEFT = 0;
    public static final double WANTED_YAW_MID = 0;
    public static final double WANTED_YAW_RIGHT = 0;
    public static final double WANTED_SKEW_LEFT = 0;
    public static final double WANTED_SKEW_MID = 0;
    public static final double WANTED_SKEW_RIGHT = 0;
    public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0;

    //math variables
    public static final int MOTOR_CONVERSION_FACTOR = 42; // 42 ticks per motor rotation, needs to be multiplied by gear                                                      // ratio
    public static final double WHEEL_DIAMTER_METERS = 0.1016;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.62; // 0.61595
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62; // 0.61595

    public static final int FRONT_LEFT_DRIVE_ID = 8; //2
    public static final int FRONT_LEFT_SPIN_ID = 5; //1
    public static final int FRONT_LEFT_CANCODER_ID = 11; //9
    public static final double FRONT_LEFT_SPIN_OFFSET_RADIANS = -Math.toRadians(150.64);//271.7578125 330 working


    public static final int FRONT_RIGHT_DRIVE_ID = 6;//4
    public static final int FRONT_RIGHT_SPIN_ID = 7;//3
    public static final int FRONT_RIGHT_CANCODER_ID = 12;//10
    public static final double FRONT_RIGHT_SPIN_OFFSET_RADIANS = -Math.toRadians(290.83); //18.193359375 110 working

    public static final int BACK_LEFT_DRIVE_ID = 4;//6
    public static final int BACK_LEFT_SPIN_ID = 3;//7
    public static final int BACK_LEFT_CANCODER_ID = 10;//12
    public static final double BACK_LEFT_SPIN_OFFSET_RADIANS = -Math.toRadians(18.19); //290.830078125 18 working c

    public static final int BACK_RIGHT_DRIVE_ID = 2;//8
    public static final int BACK_RIGHT_SPIN_ID = 1;//5
    public static final int BACK_RIGHT_CANCODER_ID = 9;//11
    public static final double BACK_RIGHT_SPIN_OFFSET_RADIANS = -Math.toRadians(271.7578125);//330.64453125 91 working c

    //swerve variables
    public static final double MAX_VOLTAGE = 12.0;
    public static final double  MAX_VELOCITY_METERS_PER_SECOND = 5800.0 / 60.0 * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
}
