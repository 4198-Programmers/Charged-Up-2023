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
    //Cancoder
    private CANCoder canCoder;
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

        canCoder = new CANCoder(canCoderID);
        configs = new CANCoderConfiguration();
        configs.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        configs.sensorDirection = Constants.CANCODER_INVERTED;
        configs.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        configs.sensorTimeBase = SensorTimeBase.PerSecond;
        configs.magnetOffsetDegrees = angleOffset;

        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        this.xFromCenter = xFromCenter;
        this.yFromCenter = yFromCenter;
        this.moduleNumber = moduleNumber;
    }
//Drive Motor Functions
    public double getDriveSpeed(){
        return driveMotor.get();
    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public void setDriveSpeed(double speed){
        driveMotor.set(speed);
    }

//Angle Motor Functions
    public double getAngleSpeed(){
        return angleMotor.get();
    }
    public double getAnglePosition(){
        return angleEncoder.getPosition();
    }
    public double getAbsolutePosition(){
        return canCoder.getAbsolutePosition();
    }
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getAbsolutePosition());
    }

    public void setAngleSpeed(double speed, double wantedAngle){
        double expectedSpeed = speed;
        if(getAbsolutePosition() > (wantedAngle + Constants.ANGLE_TOLERANCE)){
            expectedSpeed = -speed;
        }else if(getAbsolutePosition() < (wantedAngle - Constants.ANGLE_TOLERANCE)){
            expectedSpeed = speed;
        }else{
            expectedSpeed = 0;
        }
        angleMotor.set(expectedSpeed);
    }

//Other Functions
    public int getModuleNumber(){
        return moduleNumber;
    }
    public Translation2d translationFromCenter(){
        return new Translation2d(xFromCenter, yFromCenter);
    }
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveSpeed(), getRotation2d());
    }
    public SwerveModuleState getOptimizedDesiredState(SwerveModuleState desiredState){
        return SwerveModuleState.optimize(desiredState, getRotation2d());
    }
    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getRotation2d());
    }
}