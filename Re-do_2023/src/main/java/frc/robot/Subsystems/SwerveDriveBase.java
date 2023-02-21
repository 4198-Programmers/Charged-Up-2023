package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BackLeft;
import frc.robot.Constants.BackRight;
import frc.robot.Constants.FrontLeft;
import frc.robot.Constants.FrontRight;


public class SwerveDriveBase extends SubsystemBase{
   private final SwerveModule frontLeft = new SwerveModule(FrontLeft.constants);
   private final SwerveModule frontRight = new SwerveModule( FrontRight.constants);
   private final SwerveModule backLeft = new SwerveModule( BackLeft.constants);
   private final SwerveModule backRight = new SwerveModule( BackRight.constants);

   private final AHRS gyro = new AHRS(SPI.Port.kMXP);
   private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, getYaw(), new SwerveModulePosition[]{
    new SwerveModulePosition(frontLeft.getDrivePosition(), frontLeft.getAngle()),
    new SwerveModulePosition(frontRight.getDrivePosition(), frontRight.getAngle()),
    new SwerveModulePosition(backLeft.getDrivePosition(), backLeft.getAngle()),
    new SwerveModulePosition(backRight.getDrivePosition(), backRight.getAngle())}, getPose());

   public Pose2d getPose(){
    return odometer.getPoseMeters();
   }

    public Rotation2d getYaw() {
        return (Constants.invertGyro)
            ? Rotation2d.fromDegrees(360 - gyro.getYaw())
            : Rotation2d.fromDegrees(gyro.getYaw());
      }

    public void zeroHeading(){
        gyro.reset();
    }
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates =
        Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(),new SwerveModulePosition[]{
            new SwerveModulePosition(frontLeft.getDrivePosition(), frontLeft.getAngle()),
            new SwerveModulePosition(frontRight.getDrivePosition(), frontRight.getAngle()),
            new SwerveModulePosition(backLeft.getDrivePosition(), backLeft.getAngle()),
            new SwerveModulePosition(backRight.getDrivePosition(), backRight.getAngle())} , pose);
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public double BalanceDrive(){
        float pitch = gyro.getPitch();
        if(pitch > Constants.PITCH_OFFSET){
            return Constants.BALANCE_SPEED;
        }else if(pitch < -Constants.PITCH_OFFSET){
            return - Constants.BALANCE_SPEED;
        }else{
            return 0;
        }
    }
}
