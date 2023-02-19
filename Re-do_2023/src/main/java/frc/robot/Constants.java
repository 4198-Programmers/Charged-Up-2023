package frc.robot;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

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
        public static final int FIELD_ORIENTATION_BUTTON = 11,
                        ROBOT_ORIENTATION_BUTTON = 12;

        // Joystick 2 Buttons
        public static final int APRIL_TAG_LEFT_BUTTON = 3,
                        APRIL_TAG_CENTER_BUTTON = 4,
                        APRIL_TAG_RIGHT_BUTTON = 5;

        // Joystick 3 Buttons
        public static final int TOGGLE_CLAW_BUTTON = 1;

        // Joystick 4 Buttons
        public static final int LAZY_SUSAN_LEFT_BUTTON = 3,
                        LAZY_SUSAN_RIGHT_BUTTON = 4,
                        SUSAN_BRAKE_BUTTON = 11,
                        SUSAN_COAST_BUTTON = 12;

        //Turret Values
        public static final double SUSAN_CIRCUMFERENCE = 0,
                        MOTOR_ROTATIONS_PER_360_SUSAN = 104.7272727273, // gear ratio = 16 sprocket ratio = 7 + (6/11)
                        BALANCE_SPEED = 0.1;
        public static final int SUSAN_MAX_ANGLE = 630,
                        VERT_OFFSET = 0,
                        ANGLE_OFFSET = 0; // for what? [2-17]- CP
        public static final float PITCH_OFFSET = 5; // Must be a float for the gyro values

        //Auto Values
        public static final double AUTO_VERT_SPEED = 0.1;
        public static final double MAX_SPEED_METERS_PER_SECOND = 0;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;
        public static final Rotation2d DRIVE_ANGLE_OFFSET = null;
        public static final double DRIVE_ENCODER_ROTATION_TO_METER = 0;
        public static final double ANGLE_ENCODER_ROTATIONS_TO_RADIANS = 0;
        public static final boolean FRONT_LEFT_MOTOR_REVERSED = false;
        public static final boolean FRONT_LEFT_ANGLE_REVERSED = false;
        public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
        public static final int FRONT_LEFT_ABSOLUTE_ENCODER_ID = 0;
        public static final boolean FRONT_RIGHT_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_ANGLE_REVERSED = false;
        public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 0;
        public static final boolean FRONT_RIGHT_ABSOULTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_MOTOR_REVERSED = false;
        public static final boolean BACK_LEFT_ANGLE_REVERSED = false;
        public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
        public static final int BACK_LEFT_ABSOLUTE_ENCODER_ID = 0;
        public static final boolean BACK_RIGHT_MOTOR_REVERSED = false;
        public static final boolean BACK_RIGHT_ANGLE_REVERSED = false;
        public static final boolean BACK_RIGHT_ABSOULTE_ENCODER_REVERSED = false;
        public static final int BACK_RIGHT_ABSOLUTE_ENCODER_ID = 0;
        public static final double FRONT_LEFT_DISTANCE_FROM_CENTER = 0;
        public static final Rotation2d FRONT_LEFT_ANGLE_FROM_CENTER = null;
        public static final double FRONT_RIGHT_DISTANCE_FROM_CENTER = 0;
        public static final Rotation2d FRONT_RIGHT_ANGLE_FROM_CENTER = null;
        public static final double BACK_LEFT_DISTANCE_FROM_CENTER = 0;
        public static final Rotation2d BACK_LEFT_ANGLE_FROM_CENTER = null;
        public static final double BACK_RIGHT_DISTANCE_FROM_CENTER = 0;
        public static final Rotation2d BACK_RIGHT_ANGLE_FROM_CENTER = null;
        public static final BiConsumer<Double, State> THETA_CONTROLLER_CONTRAINTS = null;
        public static double driveKS;
        public static double driveKA;
        public static double kPangle;
        public static Object kPThetaController;
        public static Object kThetaControllerConstraints;
}
