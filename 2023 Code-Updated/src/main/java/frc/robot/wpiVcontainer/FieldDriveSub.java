package frc.robot.wpiVcontainer;

import com.kauailabs.navx.frc.AHRS;
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
import frc.robot.subsystems.GyroSub;

public class FieldDriveSub extends SubsystemBase {
  CANSparkMax FRdriveMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax FLdriveMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax BRdriveMotor = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax BLdriveMotor = new CANSparkMax(8, MotorType.kBrushless);
  private final FieldSwerveModule frontRSwerve = new FieldSwerveModule(FRdriveMotor, 3, 10, 27); // +27
                                                                                                           // -42.01171875
  private final FieldSwerveModule frontLSwerve = new FieldSwerveModule(FLdriveMotor, 1, 9, 32);// +32
                                                                                                        // 57.12890625
  private final FieldSwerveModule backRSwerve = new FieldSwerveModule(BRdriveMotor, 5, 11, 33.5);// +33.5
                                                                                                        // -3.251953125
  private final FieldSwerveModule backLSwerve = new FieldSwerveModule(BLdriveMotor, 7, 12, 35);// +35
                                                                                                         // 35.149921875
  RelativeEncoder FRdriveENC = FRdriveMotor.getEncoder();
  RelativeEncoder FLdriveENC = FLdriveMotor.getEncoder();
  RelativeEncoder BRdriveENC = BRdriveMotor.getEncoder();
  RelativeEncoder BLdriveENC = BLdriveMotor.getEncoder();
  Translation2d frontRLocation = new Translation2d(0.381, -0.381); // switched to match joystick inputs/orientation
  Translation2d frontLLocation = new Translation2d(0.381, 0.381);
  Translation2d backRLocation = new Translation2d(-0.381, -0.381);
  Translation2d backLLocation = new Translation2d(-0.381, 0.381);
  GyroSub gyroSub = new GyroSub();
  SwerveDriveKinematics kinematic;
  SwerveDriveOdometry odometry;

  public FieldDriveSub(GyroSub gyroSubArg) {
    gyroSub = gyroSubArg;
    kinematic = new SwerveDriveKinematics(frontRLocation, frontLLocation, backRLocation,
    backLLocation);
    odometry = new SwerveDriveOdometry(kinematic, gyroSub.getRotation2dManual(),
    new SwerveModulePosition[] {
        frontRSwerve.getPosition(), frontLSwerve.getPosition(), backRSwerve.getPosition(),
        backLSwerve.getPosition() });
  }
  
  
  
  

  double maxMPS = 0.4667056958; // max Meters per second of the drive motors (relative to the wheels)


  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldRelative) {
    
    // SwerveModuleState[] swerveModuleStatesArray = kinematic
    // .toSwerveModuleStates(fieldRelative?
    // ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed,
    // -xSpeed, zSpeed, m_gyro.getRotation2d()) : new ChassisSpeeds(-ySpeed,
    // -xSpeed, zSpeed));
    SwerveModuleState[] swerveModuleStatesArray = kinematic
        .toSwerveModuleStates(new ChassisSpeeds(-ySpeed, -xSpeed, zSpeed));
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
    odometry.update(gyroSub.getRotation2dManual(),
        new SwerveModulePosition[] { frontRSwerve.getPosition(), frontLSwerve.getPosition(),
            backRSwerve.getPosition(), backLSwerve.getPosition() });
  }

}
