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
    public SwerveSubsystem(){
        frontLeftModule = new SwerveModule(
            FrontLeftModuleConstants.DRIVE_MOTOR_ID, 
            FrontLeftModuleConstants.ANGLE_MOTOR_ID, 
            FrontLeftModuleConstants.ANGLE_ENCODER_ID, 
            FrontLeftModuleConstants.ANGLE_OFFSET_DEGREES);

        frontRightModule = new SwerveModule(
            FrontRightModuleConstants.DRIVE_MOTOR_ID, 
            FrontRightModuleConstants.ANGLE_MOTOR_ID, 
            FrontRightModuleConstants.ANGLE_ENCODER_ID, 
            FrontRightModuleConstants.ANGLE_OFFSET_DEGREES);

        backLeftModule = new SwerveModule(
            BackLeftModuleConstants.DRIVE_MOTOR_ID, 
            BackLeftModuleConstants.ANGLE_MOTOR_ID, 
            BackLeftModuleConstants.ANGLE_ENCODER_ID, 
            BackLeftModuleConstants.ANGLE_OFFSET_DEGREES);

        backRightModule = new SwerveModule(
            BackRightModuleConstants.DRIVE_MOTOR_ID, 
            BackRightModuleConstants.ANGLE_MOTOR_ID, 
            BackRightModuleConstants.ANGLE_ENCODER_ID, 
            BackRightModuleConstants.ANGLE_OFFSET_DEGREES);

        odometry = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, gyro.getRotation2d(), getModulePositions());
    }
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getModulePositions());
        // driveTab.addNumber("Front Left Angle", () -> frontLeftModule.getAngle());
        // driveTab.addNumber("Front Right Angle", () -> frontRightModule.getAngle());
        // driveTab.addNumber("Back Left Angle", () -> backLeftModule.getAngle());
        // driveTab.addNumber("Back Right Angle", () -> backRightModule.getAngle());
        // System.out.println("Front Left Angle: " + frontLeftModule.getPosition());
        // System.out.println("Front Right Angle: " + frontRightModule.getPosition());
        // System.out.println("Back Left Angle: " + backLeftModule.getPosition());
        // System.out.println("Back Right Angle: " + backRightModule.getPosition());
        //System.out.println("Angle: " +  backLeftModule.getAngle());
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
        //System.out.println("Angle: " + states[0].angle.getDegrees());
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