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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mod0;
import frc.robot.Constants.Mod1;
import frc.robot.Constants.Mod2;
import frc.robot.Constants.Mod3;

public class DriveTrain extends SubsystemBase {

        public static final double MAX_VOLTAGE = 12.0; // Not truly a max voltage, but more like a speed modifier

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 // 5880 is the max rotations of the
                                                                                  // motor per second
                        * (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0) /*Drive Reduction */
                        * 0.10033/*Wheel Diameter */ * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / // more
                                                                                                              // math
                        Math.hypot(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_LENGTH_METERS / 2.0);

        private final SwerveDriveKinematics mkinematics = new SwerveDriveKinematics( // Creates an internal drawing of
                                                                                     // the drivebase for calculation
                        // Front Left
                        new Translation2d(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                        // Front Right
                        new Translation2d(Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                        // Back Left
                        new Translation2d(-Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        Constants.DRIVETRAIN_LENGTH_METERS / 2.0),
                        // Back Right
                        new Translation2d(-Constants.DRIVETRAIN_WIDTH_METERS / 2.0,
                                        -Constants.DRIVETRAIN_LENGTH_METERS / 2.0));
        public SwerveDriveKinematics getKinematics(){
                return mkinematics;
        }

        private final AHRS NavX = new AHRS(SPI.Port.kMXP, (byte) 200); // initializes the gyro to the board port (MXP)
        private SwerveDriveOdometry swerveDriveOdometry;
        private Field2d field;

        private SwerveModulePosition[] positions = new SwerveModulePosition[]{

        };

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule backLeft;
        private final SwerveModule backRight;
        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0); // start at zeros just in case

        public DriveTrain() {
                Shuffleboard.getTab("DriveTrain");

                frontLeft =     new SwerveModule(0, Mod0.constants);
                frontRight =    new SwerveModule(1, Mod1.constants);
                backLeft =      new SwerveModule(2, Mod2.constants);
                backRight =     new SwerveModule(3, Mod3.constants);


                swerveDriveOdometry = new SwerveDriveOdometry(mkinematics, getYaw(), positions);

        }

        public void zeroGyro() { // sets the robots current front to zero in the gyro (top of the board when
                                 // verical)
                NavX.zeroYaw();
        }

        public Rotation2d getYaw(){
                return (Constants.invertGyro) ? Rotation2d.fromDegrees(360 - NavX.getYaw()): Rotation2d.fromDegrees(NavX.getYaw());
        }

        // We are passing in a boolean so that it can easily switch from field to robot
        // orientation.
        public Rotation2d getGyroRotation(boolean fieldOrientation) { // Manually returns the gyro position as a
                                                                      // Rotation2d so that wpi can use
                // it to do math for us
                // if (NavX.isMagnetometerCalibrated()) {
                // return Rotation2d.fromDegrees(-NavX.getFusedHeading());
                // }
                if (fieldOrientation) {
                        return Rotation2d.fromDegrees(-NavX.getYaw() + 90);// -NavX.getYaw so that the wheels turn in
                                                                           // the right direction + 90 so the the front
                                                                           // of the robot is considered the forward
                                                                           // direction when reset.
                } else {
                        return Rotation2d.fromDegrees(90); // This sets the front of the robot to be the front/forward
                                                           // direction at all times.
                }

        }

        public double BalanceDrive() {
                float pitch = NavX.getPitch();
                if (pitch > Constants.PITCH_OFFSET) {
                        return Constants.BALANCE_SPEED;
                } else if (pitch < -Constants.PITCH_OFFSET) {
                        return -Constants.BALANCE_SPEED;
                } else {
                        return 0;
                }
        }

        public void StopDrive(){
                ChassisSpeeds noMove = new ChassisSpeeds(0,0,0);
                drive(noMove);
        }

        public void ZeroDrive() {
                // code to zero drive
        }

        public double[] DrivePos() {
                double[] positions = { 0, 0, 0, 0 };
                return positions;
        }

        public void drive(ChassisSpeeds speeds) { // passes in speeds to be used in periodic
                chassisSpeeds = speeds;
        }

        public Pose2d getPose(){
                return swerveDriveOdometry.getPoseMeters();
        }

        @Override
        public void periodic() { // makes sure this is run every cycle of the robot
                // uses the kinematic from earlier and the wanted speeds (x,y,z) to make 4
                // unique speeds for each wheel
                SwerveModuleState[] states = mkinematics.toSwerveModuleStates(chassisSpeeds);
                // makes sure the wheels aren't passed a speed faster than they can go
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                // puts the gyro angle on the dashboard for debugging
                SmartDashboard.putNumber("Gyro Angle", NavX.getAngle());

                field.setRobotPose(getPose());
                // sets each wheel to their assigned speed from an array, this must be done in
                // the same order as the kinematic was made
                frontLeft.setDesiredState(states[0], false);
                frontRight.setDesiredState(states[1], false);
                backLeft.setDesiredState(states[2], false);
                backRight.setDesiredState(states[3], false);

        }

}
