package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Subsystems.SwerveModuleConstants;

public final class Constants {
        // Drive Train Values
        public static final double DRIVETRAIN_WIDTH_METERS = 0.62;
        public static final double DRIVETRAIN_LENGTH_METERS = 0.62;

        // Motor IDs and Pneumatics Ports
        public static final int SUSAN_MOTOR_ID = 15,
                        PNEUMATICS_PORT = 0,
                        PNEUMATICS_SINGLE_CHANNEL = 7,
                        VERTICAL_MOVER_MOTOR_ID = 14,
                        IN_OUT_MOTOR_ID = 13,
                        LED_CAN_ID = 0;

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
        public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS = 0;
        public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = 0;
        public static final double ANGLE_ENCODER_ROTATIONS_TO_RADIANS = 0;
        public static final double ANGLE_ENCODER_VELOCITY_CONVERSION = 0;
        public static final double ANGLE_KP = 0;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics( // Creates an internal drawing of the drivebase for calculation
                // Front Left
                new Translation2d(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                // Front Right
                new Translation2d(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                -Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                // Back Left
                new Translation2d(-Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                // Back Right
                new Translation2d(-Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                -Constants.DRIVETRAIN_LENGTH_METERS / 2.0));
                
        public static final double GEAR_RATIO = 0;
        public static final double WHEEL_CIRCUMFERENCE = 0;
        public static final double MAX_SPEED = 0;
        public static final double X_PID_CONTROLLER_KP = 0;
        public static final double Y_PID_CONTROLLER_KP = 0;
        public static final double THETA_PID_CONTROLLER_KP = 0;
        public static final Constraints THETA_CONTROLLER_CONSTRAINTS = null;

        public static final class FrontLeft{
                public static final int driveMotorID = 7,
                        angleMotorID = 8,
                        absoluteEncoderID = 12;
                public static final boolean driveMotorReversed = false, 
                        angleMotorReversed = false, 
                        absoluteEncoderReversed = false;
                public static final double absoluteEncoderOffsetRad = -Math.toRadians(291.18);

                public static SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, driveMotorReversed, angleMotorReversed, absoluteEncoderID,  absoluteEncoderOffsetRad, absoluteEncoderReversed);
        }

        public static final class FrontRight{
                public static final int driveMotorID = 3,
                        angleMotorID = 4,
                        absoluteEncoderID = 11;
                public static final boolean driveMotorReversed = false, 
                        angleMotorReversed = false, 
                        absoluteEncoderReversed = false;
                public static final double absoluteEncoderOffsetRad = -Math.toRadians(150 - 180);

                public static SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, driveMotorReversed, angleMotorReversed, absoluteEncoderID,  absoluteEncoderOffsetRad, absoluteEncoderReversed);
        }
        public static final class BackLeft{
                public static final int driveMotorID = 5,
                        angleMotorID = 6,
                        absoluteEncoderID = 9;
                public static final boolean driveMotorReversed = false, 
                        angleMotorReversed = false, 
                        absoluteEncoderReversed = false;
                public static final double absoluteEncoderOffsetRad = -Math.toRadians(272.63);

                public static SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, driveMotorReversed, angleMotorReversed, absoluteEncoderID,  absoluteEncoderOffsetRad, absoluteEncoderReversed);
        }

        public static final class BackRight{
                public static final int driveMotorID = 2,
                        angleMotorID = 1,
                        absoluteEncoderID = 10;
                public static final boolean driveMotorReversed = false, 
                        angleMotorReversed = false, 
                        absoluteEncoderReversed = false;
                public static final double absoluteEncoderOffsetRad = -Math.toRadians(197.75 - 180);

                public static SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, driveMotorReversed, angleMotorReversed, absoluteEncoderID,  absoluteEncoderOffsetRad, absoluteEncoderReversed);
        }

        public static boolean invertGyro = true;

}
