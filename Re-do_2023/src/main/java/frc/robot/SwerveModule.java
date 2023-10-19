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

    private double driveSpeed;
    private double angle;
    //PIDController
    /**
     * This implements a PID Control loop. <p>
     * A PID(Proportional-Integral-Derivative) control loop is used 
     * in a closed control feedback loop to regulate a process, 
     * such as the motion of a motor or the flow through a valve.<p>
     * kp - The proportional coefficient<p>
     * ki - The integral coefficient <p>
     * kd - The derivative coefficient <p>
     * period - The period between controller updates in seconds. Must be non-zero and positive (defalut is 0.02 seconds)
     */
    private PIDController angleController;
    //Module Number for the Module States array
    private int moduleNumber;
    //CANCoder
    /**This will be used to get the absolute position of the angle motor
     * and to set the initial position.
     */
    CANCoder canCoder;
    //Cancoder configs
    /**This makes sure that the CANCoder is set up
     * the way that we want it to be.
     */
    private CANCoderConfiguration configs;
    //X and Y from center
    /*These just make it easier to get the translation2d later */
    private double xFromCenter;
    private double yFromCenter;
    public SwerveModule(int driveMotorID, int angleMotorID, int canCoderID, 
    double angleOffset, double xFromCenter, double yFromCenter, int moduleNumber){
        //Creating the motors
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        //Getting the relative encoder for the drive motor
        driveEncoder = driveMotor.getEncoder();

        //Creating the CANCoder and setting up its configurations
        canCoder = new CANCoder(canCoderID);
        configs = new CANCoderConfiguration();
        //Sets the sensor range to 0-360, which is the default
        configs.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        //Indicates if the sensor is backwards
        configs.sensorDirection = Constants.CANCODER_INVERTED;
        //When the robot is first turned on, it turns the wheel to 0.
        configs.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        configs.sensorTimeBase = SensorTimeBase.PerSecond;
        //This sets the offset
        configs.magnetOffsetDegrees = angleOffset;
        canCoder.configAllSettings(configs);

        //Creating the PIDController
        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        this.xFromCenter = xFromCenter;
        this.yFromCenter = yFromCenter;
        this.moduleNumber = moduleNumber;
    }
//Drive Motor Functions
/**
 * This gets the driveSpeed that is continuously updated through the periodic
 * in SwerveSubsystem
 * @return The driveSpeed
 */
    public double getDriveSpeed(){
        return driveSpeed;
    }
/**
 * This gets the position (used in Auto possibly)
* @return Drive Position
*/
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

//Angle Motor Functions
/**
 * This gets the Angle in degress that gets updated through the periodic
 * in SwerveSubsystem
 * @return The angle in degrees
 */
    public double getAngle(){
        return angle;
    }
    public double getAbsoluteAngle(){
        return canCoder.getAbsolutePosition();
    }

//Other Functions
/**
 * This takes the updates from the periodic
 * @param driveSpeed This is what the drive speed is gotten from the desired state
 * @param angle This is what the angle is gotten from the desired state
 */
    public void set(double driveSpeed, double angle){
        this.driveSpeed = driveSpeed;
        this.angle = angle;
    }
    /**
     * This just makes things more organized and understandable
     * @return The module number for each
     */
    public int getModuleNumber(){
        return moduleNumber;
    }
    /**
     * This sets up the translation2d of each wheel module
     * @return The translation2d from the center
     */
    public Translation2d translationFromCenter(){
        return new Translation2d(xFromCenter, yFromCenter);
    }
    /**
     * SwerveModulePosition - The position of each swerve module using the drive position and the angle
     * @return The SwerveModulePosition
     */
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAngle()));
    }
    /**
     * SwerveModuleState - represents the state of each module using the drivespeed and the current angle
     * @return The current SwerveModuleState
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getAngle()));
    }
    /**
     * This is what is used to set the motor speeds in relation to the wanted state
     * We use this during the drive command to tell the motors what to do.
     * @param desiredState This is the desired state found by using the wanted angle and drivespeed
     */
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
    /**
     * This stops the wheels from moving once the drivespeed is close enough to zero.
     */
    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }
}