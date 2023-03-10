package frc.robot;

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

public class DriveTrainMod extends SubsystemBase {
        // private final AHRS gyro = new AHRS(Port.kUSB);
        public static final double MAX_VOLTAGE = 12;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0
                        * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
                        * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        // FL
                        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // FR
                        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // BL
                        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // BR
                        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);// custom update rate?

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule backLeft;
        private final SwerveModule backRight;
        private final ShuffleboardTab Tab;

        private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        public DriveTrainMod() {
                Tab = Shuffleboard.getTab("DriveTrain");
                // gyro.setAngleAdjustment(90);

                frontLeft = Mk4iSwerveModuleHelper.createNeo(
                                Tab.getLayout("FL", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.FRONT_LEFT_DRIVE_ID,
                                Constants.FRONT_LEFT_SPIN_ID,
                                Constants.FRONT_LEFT_CANCODER_ID,
                                Constants.FRONT_LEFT_SPIN_OFFSET_RADIANS);

                frontRight = Mk4iSwerveModuleHelper.createNeo(
                                Tab.getLayout("FR", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.FRONT_RIGHT_DRIVE_ID,
                                Constants.FRONT_RIGHT_SPIN_ID,
                                Constants.FRONT_RIGHT_CANCODER_ID,
                                Constants.FRONT_RIGHT_SPIN_OFFSET_RADIANS);

                backLeft = Mk4iSwerveModuleHelper.createNeo(
                                Tab.getLayout("BL", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.BACK_LEFT_DRIVE_ID,
                                Constants.BACK_LEFT_SPIN_ID,
                                Constants.BACK_LEFT_CANCODER_ID,
                                Constants.BACK_LEFT_SPIN_OFFSET_RADIANS);

                backRight = Mk4iSwerveModuleHelper.createNeo(
                                Tab.getLayout("BR", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.BACK_RIGHT_DRIVE_ID,
                                Constants.BACK_RIGHT_SPIN_ID,
                                Constants.BACK_RIGHT_CANCODER_ID,
                                Constants.BACK_RIGHT_SPIN_OFFSET_RADIANS);

        }

        public void zeroGyro() {
                gyro.zeroYaw();
        }

        public Rotation2d ManualGyroRotation() {
                if (gyro.isMagnetometerCalibrated()) {
                        return Rotation2d.fromDegrees(360 - gyro.getFusedHeading());
                } else {

                        return Rotation2d.fromDegrees(360 - gyro.getYaw());
                }
        }

        public void drive(ChassisSpeeds speedsArg) {
                speeds = speedsArg;
        }

        @Override
        public void periodic() {
              
              
              SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
              SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

              frontLeft.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
              frontRight.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
              backLeft.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
              backRight.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
              
              
              
              
              /*   SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states,
                                MAX_VELOCITY_METERS_PER_SECOND);
                frontLeft.set(states[0].speedMetersPerSecond / MAX_VOLTAGE, states[0].angle.getRadians());
                frontRight.set(states[1].speedMetersPerSecond / MAX_VOLTAGE, states[1].angle.getRadians());
                backLeft.set(states[2].speedMetersPerSecond / MAX_VOLTAGE, states[2].angle.getRadians());
                backRight.set(states[3].speedMetersPerSecond / MAX_VOLTAGE, states[3].angle.getRadians());
*/
        }

}
