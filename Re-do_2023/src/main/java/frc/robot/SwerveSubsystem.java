package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
/* Make Swerve Modules */
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;
/* Make the Gyro:
 * The first parameter is the port that is being used
 * The second parameter is the update rate in Hertz
 */
    private AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 100);
    
/* Make the SwerveKinematics */
    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        frontLeft.locationOfModule(),
        frontRight.locationOfModule(),
        backLeft.locationOfModule(),
        backRight.locationOfModule()
    );
    /**
     * This returns the kinematics
     * @return swerveKinematics
     */
    public SwerveDriveKinematics getKinematics(){
        return swerveKinematics;
    }
    /* This creates the Chassis speeds, which are originally set to 0 to prevent any problems */
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    /*
     * This makes an array of the swerveModules to help set speeds and angles
     * This allows us to use the Chassis speeds to update the states
     */
    private SwerveModule[] swerveModules;
    /*
     * This makes an array of the swerveModulePositions to use it in odometry 
     */
    //private SwerveModulePosition[] swerveModulePositions;
    /* This is used during auto to tell the robot where it is and where it needs to go to. */
    //private SwerveDriveOdometry odometry;
    boolean fieldOriented;

    public SwerveSubsystem(){
        /* Makes a new tab to organize the Swerve Drive values from the rest of the other values */
        Shuffleboard.getTab("Swerve Drive");
        /* We are now intializing the constants for the swerve modules */
        frontLeft = new SwerveModule(
        Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
        Constants.FRONT_LEFT_ANGLE_MOTOR_ID, 
        Constants.FRONT_LEFT_CANCODER_ID, 
        Constants.FRONT_LEFT_ANGLE_OFFSET, 
        Constants.FRONT_LEFT_X_FROM_CENTER, 
        Constants.FRONT_LEFT_Y_FROM_CENTER,
        Constants.FRONT_LEFT_MODULE_NUMBER);
        frontRight = new SwerveModule(
        Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
        Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
        Constants.FRONT_RIGHT_CANCODER_ID, 
        Constants.FRONT_RIGHT_ANGLE_OFFSET, 
        Constants.FRONT_RIGHT_X_FROM_CENTER, 
        Constants.FRONT_RIGHT_Y_FROM_CENTER,
        Constants.FRONT_RIGHT_MODULE_NUMBER);
        backLeft = new SwerveModule(
        Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
        Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
        Constants.BACK_LEFT_CANCODER_ID, 
        Constants.BACK_LEFT_ANGLE_OFFSET, 
        Constants.BACK_LEFT_X_FROM_CENTER, 
        Constants.BACK_LEFT_Y_FROM_CENTER,
        Constants.BACK_LEFT_MODULE_NUMBER);
        backRight = new SwerveModule(
        Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
        Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
        Constants.BACK_RIGHT_CANCODER_ID, 
        Constants.BACK_RIGHT_ANGLE_OFFSET, 
        Constants.BACK_RIGHT_X_FROM_CENTER, 
        Constants.BACK_RIGHT_Y_FROM_CENTER,
        Constants.BACK_RIGHT_MODULE_NUMBER);

        /* We are now puting the swerve modules in the array */
        swerveModules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        /* We are now putting the swerve module positions in their array */
        // swerveModulePositions = new SwerveModulePosition[]{
        //     frontLeft.getSwerveModulePosition(),
        //     frontRight.getSwerveModulePosition(),
        //     backLeft.getSwerveModulePosition(),
        //     backRight.getSwerveModulePosition()
        // };
        /* This is setting up the initial odometry.
         * The initial pose2d is not known. */
       //odometry = new SwerveDriveOdometry(swerveKinematics, getGyroRotation(true), swerveModulePositions, new Pose2d(0, 0, new Rotation2d(0, 0)));
        zeroGyro();
    }
//Gryo functions
    /**
     * Yaw - It is a measure of rotation around the Z Axis (which is perpendicular to the earth).
     * @return The current yaw value in degrees (-180 to 180).
     */
    public float getYaw(){
        return gyro.getYaw();
    }
    /**
     * Pitch - It is a measure of rotation around the X Axis.
     * @return The current pitch value in degrees (-180 to 180).
     */
    public float getPitch(){
        return gyro.getPitch();
    }
    /**
     * World linear acceleration refers to raw acceleration data, 
     * which has had the gravity component removed, 
     * and which has been rotated to the same reference frame 
     * as the current yaw value. The resulting value represents 
     * the current acceleration in the x-axis of the body 
     * (e.g., the robot) on which the sensor is mounted.
     * @return Current world linear acceleration in the X-axis (in G).
     */
    public float getXAcceleration(){
        return gyro.getWorldLinearAccelX();
    }
    /**
     * World linear acceleration refers to raw acceleration data, 
     * which has had the gravity component removed, 
     * and which has been rotated to the same reference frame 
     * as the current yaw value. The resulting value represents 
     * the current acceleration in the Y-axis of the body 
     * (e.g., the robot) on which the sensor is mounted.
     * @return Current world linear acceleration in the Y-axis (in G).
     */
    public float getYAcceleration(){
        return gyro.getWorldLinearAccelY();
    }
    /**
     * Indicates if the sensor is currently detecting motion, 
     * based upon the X and Y-axis world linear acceleration values. 
     * If the sum of the absolute values of the X and Y axis exceed 
     * a "motion threshold", the motion state is indicated.
     * @return Returns true if the sensor is currently detecting motion.
     */
    public boolean isMoving(){
        return gyro.isMoving();
    }
    /**
     * Sets the user-specified yaw offset to the current yaw value reported by the sensor.<p>
     * This user-specified yaw offset is automatically subtracted 
     * from subsequent yaw values reported by the getYaw() method. 
     * NOTE: This method has no effect if the sensor is currently 
     * calibrating, since resetting the yaw will interfere with the
     *  calibration process.
     */
    public void zeroGyro(){
        gyro.zeroYaw();
    }
    /**
     * Calibrate the gyro. It's important to make sure that the 
     * robot is not moving while the calibration is in progress,
     *  this is typically done when the robot is first turned on 
     * while it's sitting at rest before the match starts.
     */
    public void resetGyro(){
        gyro.calibrate();
    }
    /**
     * Indicates whether the sensor is currently connected to the 
     * host computer. A connection is considered established 
     * whenever communication with the sensor has occurred recently.
     * @return Returns true if a valid update has been recently received from the sensor.
     */
    public boolean gyroConnected(){
        return gyro.isConnected();
    }
    /**
     * Returns true if the sensor is currently performing automatic 
     * gyro/accelerometer calibration. Automatic calibration occurs 
     * when the sensor is initially powered on, during which time the 
     * sensor should be held still, with the Z-axis pointing up 
     * (perpendicular to the earth).
     * @return Returns true if the sensor is currently automatically 
     * calibrating the gyro and accelerometer sensors.
     */
    public boolean calibratingGyro(){
        return gyro.isCalibrating();
    }
    /**
     * The return depends on if the robot is field oriented  or robot oriented<p>
     * If it is field oriented, the return is based on the yaw of the gyro.<p>
     * If it is robot oriented, the return will always have the fron of the robot be forward.<p>
     * @param fieldOriented This is a boolean value of whether or not the robot is field oriented or not.
     * @return the Rotation2d of the robot.
     */
    public Rotation2d getGyroRotation(boolean fieldOriented){
        if(fieldOriented){
            return Rotation2d.fromDegrees(gyro.getYaw());
        }else{
            return Rotation2d.fromDegrees(0);
        }
    }
//Drive Function
    /**
    * This uses joystick inputs, converts them to chassis speeds and use that to set the drive and angle motor speeds <p>
    * <pre>
    * Robot Orientation
    *          +x(front)
    *          _________
    *          |       |
    *+y(right) |       | -y(left)
    *          |       |
    *          |_______|
    *           -x(back)
    * </pre>
    * @param x The value used for the x speed of the robot.
    * @param y The value used for the y speed of the robot.
    * @param z The value used for the z speed of the robot.
    * @param fieldOriented if the robot is field oriented or not.
    */
    public void drive(double x, double y, double z, boolean fieldOriented){
        SlewRateLimiter xyLimiter = new SlewRateLimiter(Constants.DRIVE_MAX_ACCELERATION);
        SlewRateLimiter zLimiter = new SlewRateLimiter(Constants.ANGULAR_MAX_ACCELERATION);

        x = Math.abs(x) > Constants.DEADBAND ? x : 0.0;
        y = Math.abs(y) > Constants.DEADBAND ? y : 0.0;
        z = Math.abs(z) > Constants.DEADBAND ? z : 0.0;

        x = xyLimiter.calculate(x) * Constants.DRIVE_MAX_SPEED;
        y = xyLimiter.calculate(y) * Constants.DRIVE_MAX_SPEED;
        z = zLimiter.calculate(z) * Constants.ANGULAR_MAX_SPEED;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, getGyroRotation(fieldOriented));
        this.fieldOriented = fieldOriented;
    }

    // public Pose2d getPose(){
    //     return odometry.getPoseMeters();
    // }
    // public void resetOdometry(Pose2d pose){
    //     odometry.resetPosition(getGyroRotation(fieldOriented), swerveModulePositions, pose);
    // }

    /*
     * This constantly updates the values of the swerve modules
     * This takes the current states of the periodic and uses them to find 
     * the drivespeed and the angle of each swerve module.
     * It also puts the angles of the modueles (in degreees) in the Shuffleboard.
     */
    @Override
    public void periodic() {
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DRIVE_MAX_SPEED);
        //odometry.update(getGyroRotation(fieldOriented), swerveModulePositions);
        frontLeft.set(
            states[Constants.FRONT_LEFT_MODULE_NUMBER],
            states[Constants.FRONT_LEFT_MODULE_NUMBER].speedMetersPerSecond, 
            states[Constants.FRONT_LEFT_MODULE_NUMBER].angle);
        frontRight.set(
            states[Constants.FRONT_RIGHT_MODULE_NUMBER], 
            states[Constants.FRONT_RIGHT_MODULE_NUMBER].speedMetersPerSecond, 
            states[Constants.FRONT_RIGHT_MODULE_NUMBER].angle);
        backLeft.set(
            states[Constants.BACK_LEFT_MODULE_NUMBER], 
            states[Constants.BACK_LEFT_MODULE_NUMBER].speedMetersPerSecond, 
            states[Constants.BACK_LEFT_MODULE_NUMBER].angle);
        backRight.set(
            states[Constants.BACK_RIGHT_MODULE_NUMBER], 
            states[Constants.BACK_RIGHT_MODULE_NUMBER].speedMetersPerSecond, 
            states[Constants.BACK_RIGHT_MODULE_NUMBER].angle);

        SmartDashboard.putNumber("Front Left Angle", frontLeft.getAngleDegress());
        SmartDashboard.putNumber("Front Right Angle", frontRight.getAngleDegress());
        SmartDashboard.putNumber("Back Left Angle", backLeft.getAngleDegress());
        SmartDashboard.putNumber("Back Right Angle", backRight.getAngleDegress());

        SmartDashboard.putNumber("Front Left Absolute Position", frontLeft.getAbsolutePosition());
        SmartDashboard.putNumber("Front Right Absolute Position", frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("Back Left Absolute Position", backLeft.getAbsolutePosition());
        SmartDashboard.putNumber("Back Right Absolute Position", backRight.getAbsolutePosition());
    }
}
