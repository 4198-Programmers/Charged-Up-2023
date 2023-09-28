package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveDrive.SwerveModule;

public class SwerveSubsystem extends SubsystemBase{
    /**
     * +x - forward     <p>
     * -x - backwards   <p>
     * +y - left       <p>
     * -y - right
     */
    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        //Front Left
        new Translation2d(Constants.DRIVE_TRAIN_WIDTH / 2.0, Constants.DRIVE_TRAIN_LENGTH / 2.0),
        //Front Right
        new Translation2d(Constants.DRIVE_TRAIN_WIDTH / 2.0, -Constants.DRIVE_TRAIN_LENGTH / 2.0),
        //Back Left
        new Translation2d(-Constants.DRIVE_TRAIN_WIDTH / 2.0, Constants.DRIVE_TRAIN_LENGTH / 2.0),
        //Back Right
        new Translation2d(-Constants.DRIVE_TRAIN_WIDTH / 2.0, -Constants.DRIVE_TRAIN_LENGTH / 2.0)
    );

    public SwerveDriveKinematics getKinematics(){
        return swerveKinematics;
    }
/**The gyro */
    private final AHRS NavX = new AHRS(SPI.Port.kMXP);

    //the swerve modules
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    //start at zero to make sure nothing happens
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    public SwerveSubsystem(){
        frontLeft = 
    }
}
