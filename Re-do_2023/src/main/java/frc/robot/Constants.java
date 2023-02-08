package frc.robot;

public final class Constants {

    public static final double DRIVETRAIN_WIDTH_METERS = 0.62;
    public static final double DRIVETRAIN_LENGTH_METERS = 0.62;

    public static final int FRONT_LEFT_DRIVE = 6,
            FRONT_LEFT_STEER = 7,
            FRONT_LEFT_ENCODER = 11;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(150.64);

    public static final int FRONT_RIGHT_DRIVE = 2,
            FRONT_RIGHT_STEER = 3,
            FRONT_RIGHT_ENCODER = 12;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(290.83);

    public static final int BACK_LEFT_DRIVE = 4,
            BACK_LEFT_STEER = 5,
            BACK_LEFT_ENCODER = 10;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(18.19);

    public static final int BACK_RIGHT_DRIVE = 0,
            BACK_RIGHT_STEER = 1,
            BACK_RIGHT_ENCODER = 9;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(91.75);

    public static final int SUSAN_MOTOR_ID = 0,
            PNEUMATICS_PORT = 0,
            PNEUMATICS_FORWARD_CHANNEL = 4,
            PNEUMATICS_REVERSE_CHANNEL = 5,
            VERTICAL_MOVER_MOTOR_ID = 0,
            IN_OUT_MOTOR_ID = 0;

}
