package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    //Motors
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    //Relative Encoders
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    private double driveSpeed;
    private double angle;
    //PIDController
    /**
     * This implements a PID Control loop. <p>
     * A PID(Proportional-Integral-Derivative) control loop is used in a closed control feedback loop to regulate a process, such as the motion of a motor or the flow through a valve.<p>
     * kp - The proportional coefficient<p>
     * ki - The integral coefficient <p>
     * kd - The derivative coefficient <p>
     * period - The period between controller updates in seconds. Must be non-zero and positive (defalut is 0.02 seconds)
     */
    private PIDController angleController;
    //Module Number
    private int moduleNumber;
    //offset
    //Cancoder configs
    private CANCoderConfiguration configs;
    //X and Y from center
    private double xFromCenter;
    private double yFromCenter;
    public SwerveModule(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, double xFromCenter, double yFromCenter, int moduleNumber){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        CANCoder canCoder = new CANCoder(canCoderID);
        configs = new CANCoderConfiguration();
        configs.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        configs.sensorDirection = Constants.CANCODER_INVERTED;
        configs.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        configs.sensorTimeBase = SensorTimeBase.PerSecond;
        configs.magnetOffsetDegrees = angleOffset;
        canCoder.configAllSettings(configs);

        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        this.xFromCenter = xFromCenter;
        this.yFromCenter = yFromCenter;
        this.moduleNumber = moduleNumber;
    }

    public void set(double driveSpeed, double angle){
        this.driveSpeed = driveSpeed;
        this.angle = angle;
    }
//Drive Motor Functions
    public double getDriveSpeed(){
        return driveSpeed;
    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

//Angle Motor Functions
    public double getAngle(){
        return angle;
    }

//Other Functions
    public int getModuleNumber(){
        return moduleNumber;
    }
    public Translation2d translationFromCenter(){
        return new Translation2d(xFromCenter, yFromCenter);
    }
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAngle()));
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getAngle()));
    }
    public SwerveModuleState getOptimizedDesiredState(SwerveModuleState desiredState){
        return SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
    }
    public void setDesiredState(SwerveModuleState desiredState){
        if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
        driveMotor.set(desiredState.speedMetersPerSecond);
        angleMotor.set(angleController.calculate(getAngle(), desiredState.angle.getDegrees()));
        angle = desiredState.angle.getDegrees();
    }
    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }
}