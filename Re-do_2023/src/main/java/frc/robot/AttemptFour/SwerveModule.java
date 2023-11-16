package frc.robot.AttemptFour;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AttemptFour.Constants.SwerveConstants;

public class SwerveModule {
    //Initializing angle and drive motors
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    
    //Initializing drive Encoder
    private RelativeEncoder driveEncoder;

    //Initializing angle Encoder
    private WPI_CANCoder angleEncoder;

    //Initializing angle PID Controller
    private PIDController anglePID;

    /**
     * Swerve Module 
     * @param driveMotorID CAN ID of the drive motor
     * @param angleMotorID CAN ID of the angle motor
     * @param angleEncoderID CAN ID of the angle encoder(CANCoder)
     * @param angleOffsetDegrees Angle Encoder Offset
     */
    public SwerveModule(int driveMotorID, int angleMotorID, int angleEncoderID, double angleOffsetDegrees){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        angleMotor = new CANSparkMax(angleEncoderID, MotorType.kBrushless);

        angleEncoder = new WPI_CANCoder(angleEncoderID);

        driveMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        angleEncoder.configFactoryDefault();

        angleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERTED);
        angleMotor.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        angleMotor.setIdleMode(IdleMode.kBrake);

        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        angleEncoder.configMagnetOffset(angleOffsetDegrees);
        angleEncoder.configSensorDirection(SwerveConstants.ANGLE_ENCODER_DIRECTION);
        angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        anglePID = new PIDController(SwerveConstants.ANGLE_KP, SwerveConstants.ANGLE_KI, SwerveConstants.ANGLE_KD);
        anglePID.enableContinuousInput(-180, 180);

        driveMotor.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
        driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
    }

    /**
     * Return the current state of the module
     * @return module state
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    /**
     * Return the current position of the module
     * @return module position
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    /**
     * Set Module State
     * @param state module state
     */
    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        double angleOutput = anglePID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());
        angleMotor.set(angleOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond);
    }

    /**
     * Get Module Angle
     */
    public double getAngle(){
        return getState().angle.getDegrees();
    }
    /**
     * Get Module Drive Speed
     */
    public double getDriveSpeed(){
        return getState().speedMetersPerSecond;
    }
    public double getAngleSpeed(){
        return angleEncoder.getVelocity();
    }
}
