package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

        public static final double MAX_VOLTAGE = 12.0;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0
                        * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_LENGTH_METERS / 2.0);

        private final SwerveDriveKinematics mkinematics = new SwerveDriveKinematics(
                        // Front Left
                        new Translation2d(-Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                        // Front Right
                        new Translation2d(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                        // Back Left
                        new Translation2d(-Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                        // Back Right
                        new Translation2d(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_LENGTH_METERS / 2.0));

        private final AHRS NavX = new AHRS(SPI.Port.kMXP, (byte) 200);

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule backLeft;
        private final SwerveModule backRight;
        private final ShuffleboardTab mTab;

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DriveTrain() {
                mTab = Shuffleboard.getTab("DriveTrain"); //mTab.getLayout("Front Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),

                frontLeft = Mk4iSwerveModuleHelper.createNeo(
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.FRONT_LEFT_DRIVE,
                                Constants.FRONT_LEFT_STEER,
                                Constants.FRONT_LEFT_ENCODER,
                                Constants.FRONT_LEFT_STEER_OFFSET);

                frontRight = Mk4iSwerveModuleHelper.createNeo(
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.FRONT_RIGHT_DRIVE,
                                Constants.FRONT_RIGHT_STEER,
                                Constants.FRONT_RIGHT_ENCODER,
                                Constants.FRONT_RIGHT_STEER_OFFSET);

                backLeft = Mk4iSwerveModuleHelper.createNeo(
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.BACK_LEFT_DRIVE,
                                Constants.BACK_LEFT_STEER,
                                Constants.BACK_LEFT_ENCODER,
                                Constants.BACK_LEFT_STEER_OFFSET);

                backRight = Mk4iSwerveModuleHelper.createNeo(
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.BACK_RIGHT_DRIVE,
                                Constants.BACK_RIGHT_STEER,
                                Constants.BACK_RIGHT_ENCODER,
                                Constants.BACK_RIGHT_STEER_OFFSET);
        }

        public void zeroGyro() {
                NavX.zeroYaw();
        }

        public Rotation2d getGyroRotation() {
                if (NavX.isMagnetometerCalibrated()) {
                        return Rotation2d.fromDegrees(-NavX.getFusedHeading());
                }

                return Rotation2d.fromDegrees(360 - NavX.getYaw());
        }

        public void drive(ChassisSpeeds speeds) {
                chassisSpeeds = speeds;
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = mkinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                SmartDashboard.putNumber("Gyro Angle", NavX.getAngle());

                frontLeft.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                frontRight.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                backLeft.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                backRight.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
        }

}
