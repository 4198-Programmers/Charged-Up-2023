package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder angleEncoder;

  private final PIDController anglePIDController;
  private final Rotation2d offset;

  private final CANCoder  absoluteEncoder;


  public SwerveModule(
    int driveMotorID, 
    int angleMotorID, 
    int absoluteEncoderID, 
    double absoluteEncoderOffset){
    absoluteEncoder = new CANCoder(absoluteEncoderID);

    offset = Rotation2d.fromRotations(absoluteEncoderOffset);

    CANCoderConfiguration angleEncoderConfiguration = Swerve.ctreConfigs.coderConfiguration;
    angleEncoderConfiguration.magnetOffsetDegrees = -offset.getRotations();
    absoluteEncoder.getAllConfigs(angleEncoderConfiguration);

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

   anglePIDController = new PIDController(AutoConstants.kPXController, 0, 0);
   anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

   resetEncoders();

  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }
  public double getAnglePosition(){
    return angleEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }
  public double getAngleVelocity(){
    return angleEncoder.getVelocity();
  }

  public double getAbsoluteEncoderPosition(){
    return absoluteEncoder.getAbsolutePosition();
  }

  public Rotation2d getRotation(){
    return Rotation2d.fromRotations(MathUtil.inputModulus(getAnglePosition(), -0.5, 0.5));
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), getRotation());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), getRotation());
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(getState().angle.getRadians());
  }

  public void setDesiredState(SwerveModuleState desiredState){
    if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation());
    driveMotor.set(state.speedMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond);
    angleMotor.set(anglePIDController.calculate(getAnglePosition(), MathUtil.inputModulus(state.angle.getRotations(), -0.5, 0.5)));
  }
  
//Original, but want to try the one above.
  // public void setDesiredState(SwerveModuleState state){
  //   if(Math.abs(state.speedMetersPerSecond) < 0.001){
  //     stop();
  //     return;
  //   }
  //   state = SwerveModuleState.optimize(state, getState().angle);
  //   driveMotor.set(state.speedMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond);
  //   angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRotations()));
  // }

  public void stop(){
    driveMotor.set(0);
    angleMotor.set(0);
  }


}
