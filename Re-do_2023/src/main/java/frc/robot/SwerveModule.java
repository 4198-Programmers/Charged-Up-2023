package frc.robot;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder = driveMotor.getEncoder();
  private final RelativeEncoder angleEncoder = angleMotor.getEncoder();

  private final PIDController anglePIDController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed)

}
