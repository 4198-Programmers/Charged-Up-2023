package frc.robot.AttemptFour;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public class SwerveConstants{

        public static final boolean ANGLE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_ENCODER_DIRECTION = false;
        public static final double ANGLE_KP = 0;
        public static final double ANGLE_KI = 0;
        public static final double ANGLE_KD = 0;

        public static final double DRIVE_WHEEL_DIAMETER = 1;
        public static final double DRIVE_GEAR_RATIO = 1;
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = (1/DRIVE_GEAR_RATIO/60) * DRIVE_WHEEL_DIAMETER * Math.PI;
        public static final double DRIVE_POSITION = 0;
        public static final double DRIVE_POSITION_CONVERSION_FACTOR = (1/DRIVE_GEAR_RATIO) * DRIVE_WHEEL_DIAMETER * Math.PI;

        public final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(FrontLeftModuleConstants.X_LOCATION, FrontLeftModuleConstants.Y_LOCATION),
            new Translation2d(FrontRightModuleConstants.X_LOCATION, FrontRightModuleConstants.Y_LOCATION),
            new Translation2d(BackLeftModuleConstants.X_LOCATION, BackLeftModuleConstants.Y_LOCATION),
            new Translation2d(BackRightModuleConstants.X_LOCATION, BackRightModuleConstants.Y_LOCATION)
        );

        //Robot Dimensions
        public static final double ROBOT_BASE_WIDTH_METERS = 0.62;
        public static final double ROBOT_BASE_LENGTH_METERS = 0.62;
        public static final double X_FROM_CENTER = ROBOT_BASE_LENGTH_METERS / 2;
        public static final double Y_FROM_CENTER = ROBOT_BASE_LENGTH_METERS / 2;

        public class FrontLeftModuleConstants{
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int ANGLE_ENCODER_ID = 12;
            public static final double ANGLE_OFFSET_DEGREES = 0;
            public static final double X_LOCATION = X_FROM_CENTER;
            public static final double Y_LOCATION = Y_FROM_CENTER;
        }
        public class FrontRightModuleConstants{
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int ANGLE_ENCODER_ID = 11;
            public static final double ANGLE_OFFSET_DEGREES = 0;
            public static final double X_LOCATION = X_FROM_CENTER;
            public static final double Y_LOCATION = -Y_FROM_CENTER;

        }
        public class BackLeftModuleConstants{
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int ANGLE_ENCODER_ID = 9;
            public static final double ANGLE_OFFSET_DEGREES = 0;
            public static final double X_LOCATION = -X_FROM_CENTER;
            public static final double Y_LOCATION = Y_FROM_CENTER;

        }
        public class BackRightModuleConstants{
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final int ANGLE_ENCODER_ID = 10;
            public static final double ANGLE_OFFSET_DEGREES = 0;
            public static final double X_LOCATION = -X_FROM_CENTER;
            public static final double Y_LOCATION = -Y_FROM_CENTER;

        }

    }

    public static final double VOLTAGE_COMPENSATION = 12.0;
}
