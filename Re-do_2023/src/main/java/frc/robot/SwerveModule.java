package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder angleEncoder;

  private final Rotation2d offset;

  private final PIDController anglePIDController;

  private final CANCoder  absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  final CTREConfigs configs = new CTREConfigs();


  public SwerveModule(int driveMotorID, int angleMotorID, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANCoder(absoluteEncoderID);

    offset = Rotation2d.fromDegrees(absoluteEncoderOffset);

    absoluteEncoder.configAllSettings(configs.coderConfiguration);
    absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);

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

  public double getAbsoluteEncoderPosition(){
    return absoluteEncoder.getAbsolutePosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }
  public double getAngleVelocity(){
    return angleEncoder.getVelocity();
  }

  public Rotation2d getCanCoderAngle() {
    double unsignedAngle = (Units.degreesToRadians(absoluteEncoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);
    return new Rotation2d(unsignedAngle);
}

  public double getAbsoluteEncoderRadians(){
    double angle = absoluteEncoder.getAbsolutePosition();
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(getCanCoderAngle().getRadians());
    }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond);
    angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));
  }
  public void stop(){
    driveMotor.set(0);
    angleMotor.set(0);
  }
  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      (driveEncoder.getPosition() / Constants.Swerve.driveGearRatio * 4),
      getAngle());
  }
}
