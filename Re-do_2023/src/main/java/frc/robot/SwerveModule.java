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
        private RelativeEncoder driveEncoder;
    private CANSparkMax angleMotor;
        private RelativeEncoder angleEncoder;
    //Constants gotten from the SwerveModuleStates
    private double driveSpeed;
    private Rotation2d angle;
    //Module number for the states array
    private int moduleNumber;
    /**CANCoder: <p>
     * This will be used to get the absolute position of the angle motor
     * and to set the initial position.
     */
    CANCoder canCoder;
    /**CANCoder configs: <p>
     * This makes sure that the CANCoder is set up the way that we want it to be.
     */
    private CANCoderConfiguration configs;
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
    /*X and Y from center:
    These just make it easier to get the translation2d later */
    private double xFromCenter;
    private double yFromCenter;
    /**
     * Makes the Swerve Module
     * @param driveMotorID The ID of the drive motor
     * @param angleMotorID The ID of the angle motor
     * @param cancoderID The ID of the CANCoder
     * @param angleOffset The angle offset of the CANCoder
     * @param xFromCenter The x position of the module in relation to the center of the robot
     * @param yFromCenter The y posiiton of the module in relation to the center of the robot
     * @param moduleNumber The module number of the module in the SwerveModuleStates array
     */
    public SwerveModule(int driveMotorID, int angleMotorID, int cancoderID, 
    double angleOffset, double xFromCenter, double yFromCenter, int moduleNumber){
        //Create the motors
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            driveEncoder = driveMotor.getEncoder();
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
            angleEncoder = angleMotor.getEncoder();

        //Create the cancoder
        canCoder = new CANCoder(cancoderID);
        //Set up the configs
        configs = new CANCoderConfiguration();
        configs.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        configs.sensorDirection = Constants.CANCODER_INVERTED;
        configs.initializationStrategy = SensorInitializationStrategy.BootToZero;
        configs.sensorTimeBase = SensorTimeBase.PerSecond;
        configs.magnetOffsetDegrees = angleOffset;
        canCoder.configAllSettings(configs);

        //Creating the PIDController
        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        //Creating the Constants
        this.xFromCenter = xFromCenter;
        this.yFromCenter = yFromCenter;
        this.moduleNumber = moduleNumber;

        angleEncoder.setPositionConversionFactor(Constants.ANGLE_GEAR_RATIO);
    }
/*
 * Things to alter the Swerve Module
 */
    /**
    * This is what is used to set the motor speeds in relation to the wanted state
    * We use this during the drive command to tell the motors what to do.
    * @param driveSpeed Gets the speed from the periodic
    * @param angle Gets the angle from the periodic
    * @param desiredState This is the desired state found by using the wanted angle and drivespeed
    */
    public void set(double driveSpeed, Rotation2d angle){
        this.driveSpeed = driveSpeed;
        this.angle = angle;
    }
    public void setAngleSpeed(double currentAngleDegrees, double wantedAngleDegrees){
        double speed = 0;
    }

    public void setDesiredState(SwerveModuleState desiredState){
        if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, angle);
        driveMotor.set(desiredState.speedMetersPerSecond);
        angleMotor.set(angleController.calculate(getAbsolutePosition(), desiredState.angle.getDegrees()));
    }
    /**
    * This stops the wheels from moving once the drivespeed is close enough to zero.
    */
    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }

/*
 * Things to get from the Swerve Module
 */
//Drive Funcitons
    /**
     * Uses the drivespeed from the periodic in the SwerveSubsystem.
     * @return The driveSpeed
     */
    public double getDriveSpeed(){
        return driveSpeed;
    }
    /**
     * Uses the DriveEncoder to get the position of the robot
     * @return The DrivePosition
     */
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
//Angle Functions
    public Rotation2d getAngle(){
        return angle;
    }
    public double getAngleDegress(){
        return angle.getDegrees();
    }
    public double getAngleRadians(){
        return angle.getRadians();
    }
    public double getAngleRotations(){
        return angle.getRotations();
    }
//CANCoder function
    public double getAbsolutePosition(){
        return canCoder.getAbsolutePosition();
    }
//Module Function
    /**
    * This just makes things more organized and understandable
    * @return The module number for each
    */
    public int getModuleNumber(){
        return moduleNumber;
    }
//Translation2d From Center
    /**
    * This sets up the translation2d of each wheel module
    * @return The translation2d from the center
    */
    public Translation2d locationOfModule(){
        return new Translation2d(xFromCenter, yFromCenter);
    }
//SwerveModule Functions 
    /**
    * SwerveModuleState - represents the state of each module using the drivespeed and the current angle
    * @return The current SwerveModuleState
    */
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveSpeed, angle);
    }
    /**
    * SwerveModulePosition - The position of each swerve module using the drive position and the angle
    * @return The SwerveModulePosition
    */
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), angle);
    }
}