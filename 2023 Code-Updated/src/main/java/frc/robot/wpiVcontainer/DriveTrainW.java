package frc.robot.wpiVcontainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainW extends SubsystemBase {
  CANSparkMax FRdriveMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax FLdriveMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax BRdriveMotor = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax BLdriveMotor = new CANSparkMax(8, MotorType.kBrushless);
  private final SwerveModule frontRSwerve = new SwerveModule(FRdriveMotor, 3, 10, 17.9296875);
  private final SwerveModule frontLSwerve = new SwerveModule(FLdriveMotor, 1, 9, 92.109375);
  private final SwerveModule backRSwerve = new SwerveModule(BRdriveMotor, 5, 11, 332.05078125);
  private final SwerveModule backLSwerve = new SwerveModule(BLdriveMotor, 7, 12, 291.181640625);
  RelativeEncoder FRdriveENC = FRdriveMotor.getEncoder();
  RelativeEncoder FLdriveENC = FLdriveMotor.getEncoder();
  RelativeEncoder BRdriveENC = BRdriveMotor.getEncoder();
  RelativeEncoder BLdriveENC = BLdriveMotor.getEncoder();
  Translation2d frontRLocation = new Translation2d(0.381, -0.381); // switched to match joystick inputs/orientation
  Translation2d frontLLocation = new Translation2d(0.381, 0.381);
  Translation2d backRLocation = new Translation2d(-0.381, -0.381);
  Translation2d backLLocation = new Translation2d(-0.381, 0.381);
  AnalogGyro m_gyro = new AnalogGyro(0);

  SwerveDriveKinematics kinematic = new SwerveDriveKinematics(frontRLocation, frontLLocation, backRLocation,
      backLLocation);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematic, m_gyro.getRotation2d(), new SwerveModulePosition[] {
      frontRSwerve.getPosition(), frontLSwerve.getPosition(), backRSwerve.getPosition(),
      backLSwerve.getPosition() });

  double maxMPS = 0.4667056958; // val MotorRPM/GR to get wheel rotations per minute, div
                                // by 60 for
  // sec, multiple circ in m for m per sec

  public DriveTrainW() {
    m_gyro.reset();
  }

  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStatesArray = kinematic
        .toSwerveModuleStates(fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed,
            -xSpeed, zSpeed, m_gyro.getRotation2d()) : new ChassisSpeeds(-ySpeed, -xSpeed, zSpeed)); //test 1
    // SwerveModuleState[] swerveModuleStatesArray = kinematic
    // .toSwerveModuleStates(new ChassisSpeeds(-ySpeed, -xSpeed, zSpeed)); //test 2
    // like this so that the front of the controller is the front of the robot cause
    // joysticks are weird
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStatesArray, maxMPS);
    frontRSwerve.setDesiredState(swerveModuleStatesArray[0]);// array order FR0, FL1, BR2, BL3
    frontLSwerve.setDesiredState(swerveModuleStatesArray[1]);
    backRSwerve.setDesiredState(swerveModuleStatesArray[2]);
    backLSwerve.setDesiredState(swerveModuleStatesArray[3]);
  }

  public void reset() {
    FRdriveENC.setPosition(0);
    FLdriveENC.setPosition(0);
    BRdriveENC.setPosition(0);
    BLdriveENC.setPosition(0);
  }

  public void updateOdometry() {
    odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[] { frontRSwerve.getPosition(), frontLSwerve.getPosition(),
        backRSwerve.getPosition(), backLSwerve.getPosition() });
  }

}
