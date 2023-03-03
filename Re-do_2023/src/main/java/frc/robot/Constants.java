package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // Drive Train Values
  public static final double DRIVETRAIN_WIDTH_METERS = 0.62;
  public static final double DRIVETRAIN_LENGTH_METERS = 0.62;
  // Motor IDs and Pneumatics Ports
  public static final int SUSAN_MOTOR_ID = 15,
      PNEUMATICS_PORT = 0, //Has to be 0 
      PNEUMATICS_SINGLE_CHANNEL = 7,
      VERTICAL_MOVER_MOTOR_ID = 14,
      IN_OUT_MOTOR_ID = 13,
      LED_CAN_ID = 0,
      GRABBER_MOTOR_ID = 16;
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
  public static final int FIELD_ORIENTATION_BUTTON = 11;
  public static final int ROBOT_ORIENTATION_BUTTON = 12;

  // Joystick 2 Buttons
  public static final int APRIL_TAG_RIGHT_BUTTON = 1;

  // Joystick 3 Buttons
  public static final int TOGGLE_CLAW_BUTTON = 1;

  // Joystick 4 Buttons
  public static final int SUSAN_ZERO_HEADING_BUTTON = 3,
      SUSAN_TOGGLE_BUTTON = 1;

  // Turret Values
  public static final double SUSAN_CIRCUMFERENCE = 0,
      MOTOR_ROTATIONS_PER_360_SUSAN = 104.7272727273, // gear ratio = 16 sprocket ratio = 7 + (6/11)
      BALANCE_SPEED = 0.1;
  public static final int SUSAN_MAX_ANGLE = 630,
      VERT_OFFSET = 0,
      ANGLE_OFFSET = 0; // for what? [2-17]- CP
  public static final float PITCH_OFFSET = 5; // Must be a float for the gyro values

  // Auto Values
  public static final double AUTO_VERT_SPEED = 0.1,
      MAX_VELOCITY_METERS_PER_SECOND = 4,
      MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3,
      VERT_SAFE_TO_SPIN_ENC_POS = 0,
      AUTO_SUSAN_SPEED = 0.1,
      LEFT_PLACEMENT_DEGREES = -150,
      VERT_BOTTOM_SHELF_PLACEMENT_ENC = 0,
      MID_PLACEMENT_DEGREES = 180,
      RIGHT_PLACEMENT_DEGREES = 150,
      VERT_PICKUP_POS = 0;

  public static final double VERT_ARM_NO_DROP_SPEED = 0.09;

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
  public static final double FRONT_LEFT_ANGLE_INWARD = 0;
  public static final double FRONT_RIGHT_ANGLE_INWARD = 0;
  public static final double BACK_LEFT_ANGLE_INWARD = 0;
  public static final double BACK_RIGHT_ANGLE_INWARD = 0;
  public static final double DRIVE_ROTATIONS_TO_METERS = 0;
public static final double VERT_TOP_SHELF_PLACEMENT_ENC = 0;
public static final double VERT_MIDDLE_SHELF_PLACEMENT_ENC = 0;
public static final double SUSAN_RIGHT_HEADING = 0;
public static final double SUSAN_MIDDLE_HEADING = 0;
public static final double SUSAN_LEFT_HEADING = 0;
public static final double REACH_TOP_TIME = 0;
public static final double REACH_MIDDLE_TIME = 0;
public static final double REACH_BOTTOM_TIME = 0;
public static final int GRABBER_IN_BUTTON = 9;
public static final int GRABBER_OUT_BUTTON = 8;

  public static final class Swerve {
    public static final double stickDeadband = 0.1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.62;
    public static final double wheelBase = 0.62;
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        // Front Left 
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        // Front Right
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        // Back Left
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        // Back Right
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final double angleOffset = -291.01;
      public static final boolean encoderReversed = false;
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 11;
      public static final double angleOffset = -152.05;
      public static final boolean encoderReversed = false;
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 9;
      public static final double angleOffset = -272.37;
      public static final boolean encoderReversed = false;
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 10;
      public static final double angleOffset = -17.84;
      public static final boolean encoderReversed = false;
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
