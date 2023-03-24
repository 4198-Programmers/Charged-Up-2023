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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

        public static final double MAX_VOLTAGE = 12.0; // Not truly a max voltage, but more like a speed modifier

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 // 5880 is the max rotations of the
                                                                                  // motor per second
                        * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

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

        public SwerveDriveKinematics getKinematics() {
                return mkinematics;
        }

        private final AHRS NavX = new AHRS(SPI.Port.kMXP, (byte) 200); // initializes the gyro to the board port (MXP)

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule backLeft;
        private final SwerveModule backRight;
        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0); // start at zeros just in case

        public DriveTrain() {
                Shuffleboard.getTab("DriveTrain");

                frontLeft = Mk4iSwerveModuleHelper.createNeo( // Create Neo is a function by SDS that will create the
                                                              // motors and control them for us
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

        public void zeroGyro() { // sets the robots current front to zero in the gyro (top of the board when
                                 // verical)
                NavX.zeroYaw();
        }

        public void resetGyro(){
                NavX.calibrate();
        }
        public boolean calibratingGyro(){
                return NavX.isCalibrating();
        }

        public double driveVel(){
                return frontRight.getDriveVelocity();
        }

        // We are passing in a boolean so that it can easily switch from field to robot
        // orientation.
        // true is field oriented
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

        public float getYaw() {
                return (NavX.getYaw());
        }

        public float getPitch() {
                float pitch = NavX.getPitch();
                return pitch;
        }

        public float getXAccel() {
                return NavX.getWorldLinearAccelX();
        }

        public float getYAccel() {
                return NavX.getWorldLinearAccelY();
        }

        public boolean isMoving() {
                boolean isMoving = NavX.isMoving();
                return isMoving;
        }

        public void StopDrive() {
                ChassisSpeeds noMove = new ChassisSpeeds(0, 0, 0);
                drive(noMove);
        }

        public void ZeroDrive() {
                // code to zero drive
        }

        public void drive(ChassisSpeeds speeds) { // passes in speeds to be used in periodic
                // System.out.println(getPitch());
                chassisSpeeds = speeds;
        }

        @Override
        public void periodic() { // makes sure this is run every cycle of the robot
                // uses the kinematic from earlier and the wanted speeds (x,y,z) to make 4
                // unique speeds for each wheel
                SwerveModuleState[] states = mkinematics.toSwerveModuleStates(chassisSpeeds);
                // makes sure the wheels aren't passed a speed faster than they can go
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                // puts the gyro angle on the dashboard for debugging
                SmartDashboard.putNumber("Gyro Angle", NavX.getYaw());

                // sets each wheel to their assigned speed from an array, this must be done in
                // the same order as the kinematic was made
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
