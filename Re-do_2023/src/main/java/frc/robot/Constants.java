package frc.robot;

public final class Constants {
  // Drive Train Values
  public static final double DRIVETRAIN_WIDTH_METERS = 0.62;
  public static final double DRIVETRAIN_LENGTH_METERS = 0.62;
  // Front Left Swerve Values
  public static final int FRONT_LEFT_DRIVE = 7,
      FRONT_LEFT_STEER = 8,
      FRONT_LEFT_ENCODER = 12;
  public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(291.18);
  // Front Right Swerve Values
  public static final int FRONT_RIGHT_DRIVE = 3,
      FRONT_RIGHT_STEER = 4,
      FRONT_RIGHT_ENCODER = 11;
  public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(150 - 180);
  // Back Left Swerve Values
  public static final int BACK_LEFT_DRIVE = 5,
      BACK_LEFT_STEER = 6,
      BACK_LEFT_ENCODER = 9;
  public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(272.63);
  // Back Right Swerve Values
  public static final int BACK_RIGHT_DRIVE = 2,
      BACK_RIGHT_STEER = 1,
      BACK_RIGHT_ENCODER = 10;
  public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(197.75 - 180);
  // Motor IDs and Pneumatics Ports
  public static final int SUSAN_MOTOR_ID = 15,
      PNEUMATICS_PORT = 0,
      PNEUMATICS_SINGLE_CHANNEL = 7,
      VERTICAL_MOVER_MOTOR_ID = 14,
      IN_OUT_MOTOR_ID = 13,
      LED_CAN_ID = 0;
  // Vert Arm and Reach Arm Encoder Values [Inaccurate as the vert arm drifts
  // after you turn off the robot 2-17]
  public static final double MAX_REACH = 1000,
      MIN_REACH = -1000,
      MAX_VERTICAL_POSITION = 2.16,
      MIN_VERTICAL_POSITION = 0.1,
      MAX_SUSAN_LEFT_POSITION = -37,
      MAX_SUSAN_RIGHT_POSITION = 34;

        // Joystick 1 Buttons
        public static final int APRIL_TAG_LEFT_BUTTON = 1;
        public static final int NO_SLIP_DRIVE_BUTTON = 2;
        public static final int FIELD_ORIENTATION_BUTTON = 11;
        public static final int ROBOT_ORIENTATION_BUTTON = 12;

        // Joystick 2 Buttons
        public static final int APRIL_TAG_RIGHT_BUTTON = 1;
        public static final int APRIL_TAG_TOP_LEFT_BUTTON = 7,
                                APRIL_TAG_TOP_RIGHT_BUTTON = 8,
                                APRIL_TAG_MID_LEFT_BUTTON = 9,
                                APRIL_TAG_MID_RIGHT_BUTTON = 10;

        // Joystick 3 Buttons
        public static final int TOGGLE_CLAW_BUTTON = 1,
                                REACH_OUT_BUTTON = 3, 
                                REACH_IN_BUTTON = 2, 
                                ZERO_SUSAN_HEADING_BUTTON = 4, 
                                ONE_EIGHTY_SUSAN_HEADING_BUTTON = 5;

        //Joystick 4 Button
        public static final int TOGGLE_SUSAN_DIRECTION_BUTTON = 1;

  // Pose estimation constants
  public static final double ROTATION_3D_ROLL = 0;
  public static final double ROTATION_3D_PITCH = 0;
  public static final double ROTATION_3D_YAW = 0;
  public static final double TRANSLATION_3D_X = 0;
  public static final double TRANSLATION_3D_Y = 0;
  public static final double TRANSLATION_3D_Z = 0;

        

        // Auto Values
        public static final double AUTO_VERT_SPEED = 0.15,
                        MAX_VELOCITY_METERS_PER_SECOND = 4,
                        MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3,
                        VERT_SAFE_TO_SPIN_ENC_POS = 0.6, // printed at 0.5, better safe than broken
                        AUTO_SUSAN_SPEED = 0.3,
                        LEFT_PLACEMENT_ENC_POS = 10,
                        VERT_BOTTOM_SHELF_PLACEMENT_ENC_MID = 0.603176,
                        VERT_BOTTOM_SHELF_PLACEMENT_ENC_SIDES = 0.9972,
                        MID_PLACEMENT_ENC_POS = 0,
                        RIGHT_PLACEMENT_ENC_POS = -10,
                        VERT_PICKUP_POS = 0.2,
                        AUTO_ENC_OFFSET = 0.05,
                        SUSAN_180_ENC_POS = 50;

  // Turret Values
  public static final double SUSAN_CIRCUMFERENCE = 0,
      MOTOR_ROTATIONS_PER_360_SUSAN = 104.7272727273, // gear ratio = 16 sprocket ratio = 7 + (6/11)
      BALANCE_SPEED = 0.1;
  public static final float PITCH_OFFSET = 5; // Must be a float for the gyro values

  public static final double VERT_ARM_NO_DROP_SPEED = 0.12;

  // april tags constants
  public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0.25,
      CUBE_TO_CONE_NODE_WIDTH = 21,
      END_OF_BOT_TO_TOP_NODE_LENGTH = 37,
      END_OF_BOT_TO_MID_NODE_LENGTH = 23,
      CAMERA_TO_DRIVEBASE_OFFSET = 18.75,
      LEVEL_ARM = 0,
      WANTED_DISTANCE_LEFT = 16 + CAMERA_TO_DRIVEBASE_OFFSET,
      WANTED_DISTANCE_MID = 16 + CAMERA_TO_DRIVEBASE_OFFSET,
      WANTED_DISTANCE_RIGHT = 16 + CAMERA_TO_DRIVEBASE_OFFSET,
      WANTED_YAW_LEFT = (90 - Math.tanh((16 + CAMERA_TO_DRIVEBASE_OFFSET) / 21.75)),
      WANTED_YAW_MID = 0,
      WANTED_YAW_RIGHT = -WANTED_YAW_LEFT,
      WANTED_SKEW_LEFT = 0,
      WANTED_SKEW_MID = 0,
      WANTED_SKEW_RIGHT = 0;

}
