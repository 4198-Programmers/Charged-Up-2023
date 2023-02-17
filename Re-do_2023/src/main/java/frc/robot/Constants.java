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
            PNEUMATICS_SINGLE_CHANNEL = 7,
            VERTICAL_MOVER_MOTOR_ID = 14,
            IN_OUT_MOTOR_ID = 13;
    public static final double MAX_REACH = 1000;
    public static final double MIN_REACH = -1000;
    public static final double MAX_VERTICAL_POSITION = 2.16;
    public static final double MIN_VERTICAL_POSITION = 0.1;
    public static final double MAX_SUSAN_LEFT_POSITION = -37;
    public static final double MAX_SUSAN_RIGHT_POSITION = 34;

    //april tags constants
    public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0;
    public static final double CAMERA_TO_DRIVEBASE_OFFSET = 0;
    public static final double WANTED_DISTANCE_LEFT = 29.875; //16 + CAMERA_TO_DRIVEBASE_OFFSET
    public static final double WANTED_DISTANCE_MID = 29.875; //16 + CAMERA_TO_DRIVEBASE_OFFSET
    public static final double WANTED_DISTANCE_RIGHT = 29.875; //16 + CAMERA_TO_DRIVEBASE_OFFSET
    public static final double WANTED_YAW_LEFT = (90-Math.tanh((16 + CAMERA_TO_DRIVEBASE_OFFSET) / 21.75));
    public static final double WANTED_YAW_MID = 0;
    public static final double WANTED_YAW_RIGHT = -WANTED_YAW_LEFT;
    public static final double WANTED_SKEW_LEFT = 0;
    public static final double WANTED_SKEW_MID = 0;
    public static final double WANTED_SKEW_RIGHT = 0;
    
    
    


   //Joystick 1 Buttons
        public static final int FIELD_ORIENTATION_BUTTON = 11;
        public static final int ROBOT_ORIENTATION_BUTTON = 12;
        
   //Joystick 2 Buttons
        public static final int APRIL_TAG_LEFT_BUTTON = 3;
        public static final int APRIL_TAG_CENTER_BUTTON = 4;
        public static final int APRIL_TAG_RIGHT_BUTTON = 5;

   //Joystick 3 Buttons
        public static final int TOGGLE_CLAW_BUTTON = 1;
   //Joystick 4 Buttons
        public static final int LAZY_SUSAN_LEFT_BUTTON = 3;
        public static final int LAZY_SUSAN_RIGHT_BUTTON = 4;
        public static final int SUSAN_BRAKE_BUTTON = 11;
        public static final int SUSAN_COAST_BUTTON = 12;
        public static final int SUSAN_ZERO_HEADING_BUTTON = 9;
        public static final double SUSAN_CIRCUMFERENCE = 0;
        public static final int ANGLE_OFFSET = 0;
}