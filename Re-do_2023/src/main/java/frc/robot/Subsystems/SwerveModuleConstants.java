package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class SwerveModuleConstants {
    public final CANSparkMax driveMotor;
    public final CANSparkMax angleMotor;

    public  final RelativeEncoder driveEncoder;
    public  final RelativeEncoder angleEncoder;

    public  final PIDController anglePIDController;

    public  AnalogInput absoluteEncoder;
    public  boolean absoluteEncoderReversed;
    public  double absoluteEncoderOffsetRad;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed, int absoluteEncoderID, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        
        driveMotor.setInverted(driveMotorReversed);
        angleMotor.setInverted(angleMotorReversed);

        driveEncoder.setPositionConversionFactor(Constants.DRIVE_ENCODER_ROTATIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(Constants.DRIVE_ENCODER_VELOCITY_CONVERSION);
        angleEncoder.setPositionConversionFactor(Constants.ANGLE_ENCODER_ROTATIONS_TO_RADIANS);
        angleEncoder.setVelocityConversionFactor(Constants.ANGLE_ENCODER_VELOCITY_CONVERSION);
        
        anglePIDController = new PIDController(Constants.ANGLE_KP, 0, 0);
        anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
}
}