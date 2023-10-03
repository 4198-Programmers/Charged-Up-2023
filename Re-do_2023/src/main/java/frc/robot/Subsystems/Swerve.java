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
  
  private final SwerveModule frontLeft = Mod0.module;
  private final SwerveModule frontRight = Mod1.module;
  private final SwerveModule backLeft = Mod2.module;
  private final SwerveModule backRight = Mod3.module;

  private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  private final SwerveModulePosition[] positions = new SwerveModulePosition[]{
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  };
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRotation2d(), positions);

  public void zeroGyro(){
    gyro.zeroYaw();
  }
  public void resetGyro(){
    gyro.calibrate();
  }
  public boolean calibratingGyro(){
    return gyro.isCalibrating();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading() - (45/360));
  }

  public Rotation2d getGyroRotation(boolean fieldOrientation){
    if (fieldOrientation){
      return Rotation2d.fromDegrees(gyro.getYaw());
    }
    else{
      return Rotation2d.fromDegrees(0);
    }
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
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

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), positions, pose);
  }

  public void printModuleAngles(){
    System.out.println("Front Left Module: " + frontLeft.getAbsoluteEncoderPosition());
    System.out.println("Front Right Module: " + frontRight.getAbsoluteEncoderPosition());
    System.out.println("Back Left Module: " + backLeft.getAbsoluteEncoderPosition());
    System.out.println("Back Right Module: " + backRight.getAbsoluteEncoderPosition());
  }


  public double balance(){
    double speed;
    if(gyro.getPitch() > 0.5){
      speed = Constants.BALANCE_SPEED;
    }
    else if(gyro.getPitch() < -0.5){
      speed = -Constants.BALANCE_SPEED;
    }
    else{
      speed = 0;
    }
    return speed;
  }

  @Override
  public void periodic() {
      odometer.update(getRotation2d(), positions);
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
      SmartDashboard.putNumber("Front Left Angle", frontLeft.getAbsoluteEncoderPosition());
      SmartDashboard.putNumber("Front Right Angle", frontRight.getAbsoluteEncoderPosition());
      SmartDashboard.putNumber("Back Left Angle", backLeft.getAbsoluteEncoderPosition());
      SmartDashboard.putNumber("Back Right Angle", backRight.getAbsoluteEncoderPosition());
      
  }
}