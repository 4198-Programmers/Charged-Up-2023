package frc.robot.Deprecated;
/**
package frc.robot.wpiDrive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldDriveSub extends SubsystemBase {
  CANSparkMax FRdriveMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax FLdriveMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax BRdriveMotor = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax BLdriveMotor = new CANSparkMax(8, MotorType.kBrushless);
  private final FieldSwerveModule frontRSwerve = new FieldSwerveModule(FRdriveMotor, 3, 10, -15.01171875); // +27
                                                                                                           // -42.01171875
                                                                                                           // og
  private final FieldSwerveModule frontLSwerve = new FieldSwerveModule(FLdriveMotor, 1, 9, 89.12890625);// +32
                                                                                                        // 57.12890625
                                                                                                        // og
  private final FieldSwerveModule backRSwerve = new FieldSwerveModule(BRdriveMotor, 5, 11, 30.24804688);// +33.5
                                                                                                        // -3.251953125
                                                                                                        // og
  private final FieldSwerveModule backLSwerve = new FieldSwerveModule(BLdriveMotor, 7, 12, 70.149921875);// +35
                                                                                                         // 35.149921875
                                                                                                         // og
  RelativeEncoder FRdriveENC = FRdriveMotor.getEncoder();
  RelativeEncoder FLdriveENC = FLdriveMotor.getEncoder();
  RelativeEncoder BRdriveENC = BRdriveMotor.getEncoder();
  RelativeEncoder BLdriveENC = BLdriveMotor.getEncoder();
  Translation2d frontRLocation = new Translation2d(0.381, -0.381); // switched to match joystick inputs/orientation
  Translation2d frontLLocation = new Translation2d(0.381, 0.381);
  Translation2d backRLocation = new Translation2d(-0.381, -0.381);
  Translation2d backLLocation = new Translation2d(-0.381, 0.381);
  AHRS gyro = new AHRS(Port.kMXP);

  SwerveDriveKinematics kinematic = new SwerveDriveKinematics(frontLLocation, frontRLocation, backLLocation,
      backRLocation);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematic, gyro.getRotation2d(),
      new SwerveModulePosition[] {
          frontLSwerve.getPosition(), frontRSwerve.getPosition(), backLSwerve.getPosition(),
          backRSwerve.getPosition() });

  // double maxMPS = 0.4667056958; // max Meters per second of the drive motors
  // (relative to the wheels)
  double maxMPS = 3.5; // max Meters per second of the drive motors (relative to the wheels)

  public void displayGyro() {
    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
    // Shuffleboard.getTab("Gyro Tab").add(gyro);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStatesArray = kinematic
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
            ySpeed, zSpeed, gyro.getRotation2d())
            : new ChassisSpeeds(-ySpeed,
                -xSpeed, zSpeed));
    // SwerveModuleState[] swerveModuleStatesArray = kinematic
    // .toSwerveModuleStates(new ChassisSpeeds(-ySpeed,-xySpeed, zSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStatesArray, maxMPS);
    frontLSwerve.setDesiredState(swerveModuleStatesArray[0]);// array order FL0, FR1, BL2, BR3
    frontRSwerve.setDesiredState(swerveModuleStatesArray[1]);
    backLSwerve.setDesiredState(swerveModuleStatesArray[2]);
    backRSwerve.setDesiredState(swerveModuleStatesArray[3]);
  }

  public void reset() {
    FRdriveENC.setPosition(0);
    FLdriveENC.setPosition(0);
    BRdriveENC.setPosition(0);
    BLdriveENC.setPosition(0);
    gyro.reset();
  }

  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(),
        new SwerveModulePosition[] { frontLSwerve.getPosition(), frontRSwerve.getPosition(),
            backLSwerve.getPosition(), backRSwerve.getPosition() });
  }

}
*/