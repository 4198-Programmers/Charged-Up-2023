package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.AutoConstants;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder angleEncoder;

  private final PIDController anglePIDController;

  private final CANCoder  absoluteEncoder;
  final CTREConfigs configs = new CTREConfigs();


  public SwerveModule(int driveMotorID, int angleMotorID, int absoluteEncoderID, double absoluteEncoderOffset){
    absoluteEncoder = new CANCoder(absoluteEncoderID);


    absoluteEncoder.configAllSettings(configs.coderConfiguration);
    absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

   anglePIDController = new PIDController(AutoConstants.kPXController, 0, 0);
   anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

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

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(getAbsoluteEncoderPosition());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    driveMotor.set(desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed);
    angleMotor.set(anglePIDController.calculate(getAnglePosition(), desiredState.angle.getRotations()));
  }

  public void stop(){
    driveMotor.set(0);
    angleMotor.set(0);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

}
