package frc.robot;

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
    public static final double WANTED_DISTANCE = 0;
    public static final double WANTED_YAW = 0;
    public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0;
    public static final double WANTED_SKEW = 0;
    
    //math variables
    public static final int MOTOR_CONVERSION_FACTOR = 42; // 42 ticks per motor rotation, needs to be multiplied by gear                                                      // ratio
    public static final double WHEEL_DIAMTER_METERS = 0.1016;
}
