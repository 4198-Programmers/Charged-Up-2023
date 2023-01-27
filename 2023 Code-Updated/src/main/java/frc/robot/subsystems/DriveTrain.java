package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public static final double maxAngularVelocityInRadiansPerSecond = Constants.MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(Constants.DRIVETRAIN_SIDE_TO_SIDE_WIDTH / 2.0, Constants.DRIVETRAIN_FRONT_TO_BACK_LENGTH / 2.0);

  private SwerveDriveKinematics swerveDrive = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Constants.DRIVETRAIN_SIDE_TO_SIDE_WIDTH / 2.0, Constants.DRIVETRAIN_FRONT_TO_BACK_LENGTH / 2.0),
      // Front right
      new Translation2d(Constants.DRIVETRAIN_SIDE_TO_SIDE_WIDTH / 2.0,
          -Constants.DRIVETRAIN_FRONT_TO_BACK_LENGTH / 2.0),
      // Back left
      new Translation2d(-Constants.DRIVETRAIN_SIDE_TO_SIDE_WIDTH / 2.0,
          Constants.DRIVETRAIN_FRONT_TO_BACK_LENGTH / 2.0),
      // Back right
      new Translation2d(-Constants.DRIVETRAIN_SIDE_TO_SIDE_WIDTH / 2.0,
          -Constants.DRIVETRAIN_FRONT_TO_BACK_LENGTH / 2.0));

      private final GyroSub gyro = new GyroSub();
      
      //swerve modules
      private final SwerveModule frontLeft;
      private final SwerveModule frontRight;
      private final SwerveModule backLeft;
      private final SwerveModule backRight;
      private final ShuffleboardTab tab;

      private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0,0.0,0.0);

  public void Move(double angle, double speed) {

  }
}