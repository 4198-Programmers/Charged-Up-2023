package frc.robot;

public final class Constants {

    public static final int MOTOR_CONVERSION_FACTOR = 42; // 42 ticks per motor rotation, needs to be multiplied by gear
                                                          // ratio
    public static final double WHEEL_DIAMTER_METERS = 0.1016;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.62; // 0.61595
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62; // 0.61595

    public static final int FRONT_LEFT_DRIVE_ID = 8; //2
    public static final int FRONT_LEFT_SPIN_ID = 5; //1
    public static final int FRONT_LEFT_CANCODER_ID = 11; //9
    public static final double FRONT_LEFT_SPIN_OFFSET_RADIANS = -Math.toRadians(330.64453125);//271.7578125 330 working


    public static final int FRONT_RIGHT_DRIVE_ID = 6;//4
    public static final int FRONT_RIGHT_SPIN_ID = 7;//3
    public static final int FRONT_RIGHT_CANCODER_ID = 12;//10
    public static final double FRONT_RIGHT_SPIN_OFFSET_RADIANS = -Math.toRadians(110.830078125); //18.193359375 110 working

    public static final int BACK_LEFT_DRIVE_ID = 4;//6
    public static final int BACK_LEFT_SPIN_ID = 3;//7
    public static final int BACK_LEFT_CANCODER_ID = 10;//12
    public static final double BACK_LEFT_SPIN_OFFSET_RADIANS = -Math.toRadians(198.193359375); //290.830078125 18 working c

    public static final int BACK_RIGHT_DRIVE_ID = 2;//8
    public static final int BACK_RIGHT_SPIN_ID = 1;//5
    public static final int BACK_RIGHT_CANCODER_ID = 9;//11
    public static final double BACK_RIGHT_SPIN_OFFSET_RADIANS = -Math.toRadians(271.7578125);//330.64453125 91 working c
}
