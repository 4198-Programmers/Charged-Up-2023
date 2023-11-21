package frc.robot.AttemptFour;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AttemptFour.Constants.SwerveConstants.BackLeftModuleConstants;
import frc.robot.AttemptFour.Constants.SwerveConstants.BackRightModuleConstants;
import frc.robot.AttemptFour.Constants.SwerveConstants.FrontLeftModuleConstants;
import frc.robot.AttemptFour.Constants.SwerveConstants.FrontRightModuleConstants;

public class SwerveSubsystem extends SubsystemBase{
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private SwerveDriveOdometry odometry;
    public ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    private final SwerveModule[] modules;
    public SwerveSubsystem(){
        frontLeftModule = new SwerveModule(
            FrontLeftModuleConstants.NAME,
            FrontLeftModuleConstants.MODULE_NUMBER,
            FrontLeftModuleConstants.DRIVE_MOTOR_ID, 
            FrontLeftModuleConstants.ANGLE_MOTOR_ID, 
            FrontLeftModuleConstants.ANGLE_ENCODER_ID, 
            FrontLeftModuleConstants.ANGLE_OFFSET_DEGREES);

        frontRightModule = new SwerveModule(
            FrontRightModuleConstants.NAME,
            FrontRightModuleConstants.MODULE_NUMBER,
            FrontRightModuleConstants.DRIVE_MOTOR_ID, 
            FrontRightModuleConstants.ANGLE_MOTOR_ID, 
            FrontRightModuleConstants.ANGLE_ENCODER_ID, 
            FrontRightModuleConstants.ANGLE_OFFSET_DEGREES);

        backLeftModule = new SwerveModule(
            BackLeftModuleConstants.NAME,
            BackLeftModuleConstants.MODULE_NUMBER,
            BackLeftModuleConstants.DRIVE_MOTOR_ID, 
            BackLeftModuleConstants.ANGLE_MOTOR_ID, 
            BackLeftModuleConstants.ANGLE_ENCODER_ID, 
            BackLeftModuleConstants.ANGLE_OFFSET_DEGREES);

        backRightModule = new SwerveModule(
            BackRightModuleConstants.NAME,
            BackRightModuleConstants.MODULE_NUMBER,
            BackRightModuleConstants.DRIVE_MOTOR_ID, 
            BackRightModuleConstants.ANGLE_MOTOR_ID, 
            BackRightModuleConstants.ANGLE_ENCODER_ID, 
            BackRightModuleConstants.ANGLE_OFFSET_DEGREES);

        odometry = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, gyro.getRotation2d(), getModulePositions());

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        for(SwerveModule module : modules){
            driveTab.addNumber(module.getName() + " Drive Speed:", () -> module.getDriveSpeed());
            driveTab.addNumber(module.getName() + " Angle Speed:", () -> module.getAngleSpeed());
            driveTab.addNumber(module.getName() + " Current Angle:", () -> module.getAngle());
        }
    }
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getModulePositions());
    }

    /**
     * Drives the swerve - Input range: [-1, 1]
     * @param xSpeed Percent Power in the X direction
     * @param ySpeed Percent Power in the Y direction
     * @param zSpeed Percent Power in the Z direction
     * @param fieldOriented Configure robot movement style (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
        SwerveModuleState[] states = null;
        if(fieldOriented){
            states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, gyro.getRotation2d()));
        }else{
            states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        System.out.println("Back Left Wanted Angle: " + states[0].angle.getDegrees());
        System.out.println("Back Left Optimized Wanted Angle: " + SwerveModuleState.optimize(states[0], backLeftModule.getState().angle));
        System.out.println("Back Left Current Angle: " + backLeftModule.getAngle());
        System.out.println("Back Left Speed: " + backLeftModule.getAngleSpeed());

        for(SwerveModule module : modules){
            System.out.println(module.getName() + " Drive Speed: " + module.getDriveSpeed());
            System.out.println(module.getName() + " Angle Speed: " + module.getAngleSpeed());
            System.out.println(module.getName() + " Current Angle: " + module.getAngle());
            System.out.println(module.getName() + " Wanted Angle: " + states[module.getModuleNumber()].angle.getDegrees());
            System.out.println(module.getName() + " Optimized Wanted Angle: " + SwerveModuleState.optimize(states[module.getModuleNumber()], module.getState().angle));

        }
        setModuleStates(states);
    }

    /**
     * Get current swerve module states
     * @return swerve module states
     */

    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    /**
     * Get current swerve module positions
     * @return swerve module positions
     */
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    /**
     * Sets swerve module states
     * @param desiredStates array of desired states, order: [frontLeft, frontRight, backLeft, backRight]
     */

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        frontLeftModule.setState(desiredStates[0]);
        frontRightModule.setState(desiredStates[1]);
        backLeftModule.setState(desiredStates[2]);
        backRightModule.setState(desiredStates[3]);
    }

    /**
     * Get predicted pose
     * @return pose
     */
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    /**
     * Set robot pose
     * @param pose robot pose
     */
    public void setPose(Pose2d pose){
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }
}