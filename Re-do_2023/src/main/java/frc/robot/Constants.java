package frc.robot;

public final class Constants {

    public static final double DRIVETRAIN_WIDTH_METERS = 0.62;
    public static final double DRIVETRAIN_LENGTH_METERS = 0.62;

    public static final int FRONT_LEFT_DRIVE = 7,
            FRONT_LEFT_STEER = 8,
            FRONT_LEFT_ENCODER = 12;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(291.18);

    public static final int FRONT_RIGHT_DRIVE = 3,
            FRONT_RIGHT_STEER = 4,
            FRONT_RIGHT_ENCODER = 11;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(150 - 180);

    public static final int BACK_LEFT_DRIVE = 5,
            BACK_LEFT_STEER = 6,
            BACK_LEFT_ENCODER = 9;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(272.63);

    public static final int BACK_RIGHT_DRIVE = 2,
            BACK_RIGHT_STEER = 1,
            BACK_RIGHT_ENCODER = 10;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(197.75 - 180);

    public static final int SUSAN_MOTOR_ID = 15,
            PNEUMATICS_PORT = 0,
            PNEUMATICS_FORWARD_CHANNEL = 4,
            PNEUMATICS_REVERSE_CHANNEL = 5,
            VERTICAL_MOVER_MOTOR_ID = 0,
            IN_OUT_MOTOR_ID = 15;
    public static final double MAX_REACH = 1000;
    public static final double MIN_REACH = -1000;
    public static final double MAX_VERTICAL_POSITION = 1000;
    public static final double MIN_VERTICAL_POSITION = -1000;
    public static final double MAX_SUSAN_LEFT_POSITION = -37;
    public static final double MAX_SUSAN_RIGHT_POSITION = 34;

    //april tags constants
    public static final double WANTED_DISTANCE_LEFT = 0;
    public static final double WANTED_DISTANCE_MID = 0;
    public static final double WANTED_DISTANCE_RIGHT = 0;
    public static final double WANTED_YAW_LEFT = 0;
    public static final double WANTED_YAW_MID = 0;
    public static final double WANTED_YAW_RIGHT = 0;
    public static final double WANTED_SKEW_LEFT = 0;
    public static final double WANTED_SKEW_MID = 0;
    public static final double WANTED_SKEW_RIGHT = 0;
    public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0;
    


   //Left Joystick Buttons
        
        
   //Mid Joystick Buttons
        public static final int APRIL_TAG_LEFT_BUTTON = 3;
        public static final int APRIL_TAG_CENTER_BUTTON = 4;
        public static final int APRIL_TAG_RIGHT_BUTTON = 5;

   //Right Joystick Buttons
        public static final int REACH_OUT_BUTTON = 5;
        public static final int REACH_IN_BUTTON = 3;
        public static final int CLAW_OPEN_BUTTON = 0;
        public static final int CLAW_CLOSE_BUTTON = 0;

}
