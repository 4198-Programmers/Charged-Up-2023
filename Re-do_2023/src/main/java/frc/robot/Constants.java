package frc.robot;

public final class Constants {
        // Drive Train Values
        public static final double DRIVETRAIN_WIDTH_METERS = 0.62;
        public static final double DRIVETRAIN_LENGTH_METERS = 0.62;
        // Front Left Swerve Values
        public static final int FRONT_LEFT_DRIVE = 7,
                        FRONT_LEFT_STEER = 8,
                        FRONT_LEFT_ENCODER = 12;
        public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(291.1816467335055);

        // Front Right Swerve Values
        public static final int FRONT_RIGHT_DRIVE = 3,
                        FRONT_RIGHT_STEER = 4,
                        FRONT_RIGHT_ENCODER = 11;
        public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(329.7656585624743);

        // Back Left Swerve Values
        public static final int BACK_LEFT_DRIVE = 5,
                        BACK_LEFT_STEER = 6,
                        BACK_LEFT_ENCODER = 9;
        public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(272.4609451619576);

        // Back Right Swerve Values
        public static final int BACK_RIGHT_DRIVE = 2,
                        BACK_RIGHT_STEER = 1,
                        BACK_RIGHT_ENCODER = 10;
        public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(17.31445270635416);

        // Motor IDs and Pneumatics Ports
        public static final int SUSAN_MOTOR_ID = 15,
                        PNEUMATICS_PORT = 0,
                        PNEUMATICS_SINGLE_CHANNEL = 7,
                        VERTICAL_MOVER_MOTOR_ID = 14,
                        IN_OUT_MOTOR_ID = 13,
                        LED_CAN_ID = 0,
                        INTAKE_MOTOR_ID = 16;

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
        public static final int AUTO_LOCK_LEFT_BTN = 1;
        public static final int BALANCE_BUTTON = 7;
        public static final int TARGET_TEST_BUTTON = 8;
        public static final int FIELD_ORIENTATION_BUTTON = 11;
        public static final int ROBOT_ORIENTATION_BUTTON = 12;

        // Joystick 2 Buttons
        public static final int APRIL_TAG_RIGHT_BUTTON = 1;
        public static final int AUTO_LOCK_RIGHT_BTN = 1;
        public static final int NO_SLIP_DRIVE_BUTTON = 2;
        public static final int TEST_SUSAN_PHOTON = 7;
        public static final int TEST_DRIVE_CENTER_PHOTON = 8;
        public static final int TEST_ZERO_DRIVE_HEADING_BUTTON = 9;
        public static final int APRIL_TAG_TEST_BUTTON = 11;
        public static final int CHANGE_DRIVE_QUARTER_SPEED_BUTTON = 3;
        public static final int CHANGE_DRIVE_FULL_SPEED_BUTTON = 5;

        // Joystick 3 Buttons
        public static final int TOGGLE_CLAW_BUTTON = 1,
                        REACH_OUT_BUTTON = 5,
                        REACH_IN_BUTTON = 3,
                        OUTTAKE_BUTTON = 4,
                        INTAKE_BUTTON = 1,
                        SUSTATION_UP_BUTTON = 2,
                        ZERO_SUSAN_BUTTON = 6,
                        PLACE_TOP_LEFT_BTN = 10,
                        PLACE_TOP_RIGHT_BTN = 9,
                        PLACE_MID_LEFT_BTN = 8,
                        PLACE_MID_RIGHT_BIN = 7,
                        SLOW_SUSAN_BUTTON = 11,
                        STRAIGHT_DOWN_INTAKE_BUTTON = 12;

        // Turret Values
        public static final double SUSAN_CIRCUMFERENCE = 0,
                        MOTOR_ROTATIONS_PER_360_SUSAN = 104.7272727273, // gear ratio = 16 sprocket ratio = 7 + (6/11)
                        BALANCE_SPEED = 0.75;
        public static final float PITCH_OFFSET = 5; // Must be a float for the gyro values

        // Auto Values
        public static final double AUTO_VERT_SPEED = 1,
                        MAX_VELOCITY_METERS_PER_SECOND = 4,
                        MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3,
                        VERT_SAFE_TO_SPIN_ENC_POS = 3.67, // printed at 0.5, better safe than broken
                        AUTO_SUSAN_SPEED = 0.15,
                        LEFT_PLACEMENT_ENC_POS = 6.0476,
                        VERT_MID_SHELF_PLACEMENT_ENC_MID = 6.5,
                        VERT_MID_SHELF_PLACEMENT_ENC_SIDES = 7,
                        MID_PLACEMENT_ENC_POS = 0,
                        RIGHT_PLACEMENT_ENC_POS = -6.0476,
                        VERT_PICKUP_POS = 2.75,
                        AUTO_ENC_OFFSET = 0.05,
                        SUSAN_180_ENC_POS = 56,
                        INTAKE_OUT_SPEED = -0.7,
                        INTAKE_IN_SPEED = 0.7,
                        AUTO_REACH_SPEED = 1,
                        VISIBLE_TAG_VERT_ENC = 4,
                        SUSAN_LEFT_FROM_TAG_ENC = 6,
                        SUBSTATION_REACH_POS = 60;

        // Auto AprilTags Values
        public static final double DISTANCE_FROM_TAG = 0,
                        PHOTON_TOLERANCE_VALUE = 0.05,
                        ANGLE_SKEW = 0;

        public static final double VERT_ARM_NO_DROP_SPEED = 0.03;

        // april tags constants
        public static final double CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE = 0.25,
                        CAMERA_TO_DRIVEBASE_OFFSET = 14,
                        WANTED_DISTANCE_LEFT = 16 + CAMERA_TO_DRIVEBASE_OFFSET,
                        WANTED_DISTANCE_MID = 16 + CAMERA_TO_DRIVEBASE_OFFSET,
                        WANTED_DISTANCE_RIGHT = 16 + CAMERA_TO_DRIVEBASE_OFFSET,
                        WANTED_YAW_LEFT = (90 - Math.tanh((16 + CAMERA_TO_DRIVEBASE_OFFSET) / 21.75)),
                        WANTED_YAW_MID = 0,
                        WANTED_YAW_RIGHT = -WANTED_YAW_LEFT,
                        WANTED_SKEW_LEFT = 0,
                        WANTED_SKEW_MID = 0,
                        WANTED_SKEW_RIGHT = 0;
        public static final double REACH_ENCODER_WHEEL_DIAMETER = 0.25,
                        REACH_ENCODER_TOLERANCE = 25,
                        TOP_REACH_LEFT_PLACEMENT = 3535,
                        TOP_REACH_RIGHT_PLACEMENT = 4250,
                        PLACE_TOP_VERT = 8.15,
                        PLACE_MID_VERT = 7.05,
                        MID_REACH_PLACEMENT = 0,
                        RIGHT_MID_PLACEMENT_SUSAN = -5.57,
                        LEFT_MID_PLACEMENT_SUSAN = 6.64,
                        RIGHT_TOP_PLACEMENT_SUSAN = -4,
                        LEFT_TOP_PLACEMENT_SUSAN = 4,
                        SUBSTATION_UP_POS_VERT = 7.92;
        public static final int SUSAN_SENSOR_CHANNEL = 0;

}
