package frc.robot.Subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

public class Swerve extends SubsystemBase {
private final SwerveModule frontLeft = new SwerveModule(
  Mod0.driveMotorID, 
  Mod0.angleMotorID, 
  Mod0.canCoderID, 
  Mod0.angleOffset, false);

  private final SwerveModule frontRight = new SwerveModule(
  Mod1.driveMotorID, 
  Mod1.angleMotorID, 
  Mod1.canCoderID, 
  Mod1.angleOffset, false);
  private final SwerveModule backLeft = new SwerveModule(
  Mod2.driveMotorID, 
  Mod2.angleMotorID, 
  Mod2.canCoderID, 
  Mod2.angleOffset, false);

  private final SwerveModule backRight = new SwerveModule(
  Mod3.driveMotorID, 
  Mod3.angleMotorID, 
  Mod3.canCoderID, 
  Mod3.angleOffset, false);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveModulePosition[] positions = new SwerveModulePosition[]{
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  };
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRotation2d(), positions);

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), positions, pose);
  }

  @Override
  public void periodic() {
      odometer.update(getRotation2d(), positions);
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
      SmartDashboard.putNumber("Front Left Angle", frontLeft.getAbsoluteEncoderRadians());
      SmartDashboard.putNumber("Front Right Angle", frontRight.getAbsoluteEncoderRadians());
      SmartDashboard.putNumber("Back Left Angle", backLeft.getAbsoluteEncoderRadians());
      SmartDashboard.putNumber("Back Right Angle", backRight.getAbsoluteEncoderRadians());
      
  }
  public void stopModules(){
   frontLeft.stop();
   frontRight.stop();
   backLeft.stop();
   backRight.stop();     
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
