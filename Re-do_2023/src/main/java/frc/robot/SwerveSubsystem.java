package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    /**Chassis Speeds - These take joystick values and translates them into speeds for the robot. */
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
/**This passes in speeds to be used in periodic to set the speed */
    public void drive(ChassisSpeeds speeds){
        this.speeds = speeds;
    }

    public Rotation2d getRotation(boolean fieldOriented){
        if(fieldOriented){
            return Rotation2d.fromDegrees(NavX.getYaw());
        }
        else{
            return Rotation2d.fromDegrees(0);
        }
    }
/** Takes the current angle and the wanted angle and returns
 * the new optimized state that can be used as a setpoint.
 */
    public SwerveModuleState optimizeState(SwerveModuleState state, Rotation2d currentAngle){
        return SwerveModuleState.optimize(state, currentAngle);
    }

    public void stopSpeed(){
        speeds = new ChassisSpeeds(0, 0, 0);
    }
    @Override
    public void periodic() {
        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];
    }



}
