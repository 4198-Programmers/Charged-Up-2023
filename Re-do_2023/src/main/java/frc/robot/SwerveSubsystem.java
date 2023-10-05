package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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


}
