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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class DriveTrain extends SubsystemBase {

        private final SwerveModule frontLeft =  new SwerveModule(
                Constants.FRONT_LEFT_DRIVE, 
                Constants.FRONT_LEFT_STEER, 
                Constants.FRONT_LEFT_MOTOR_REVERSED, 
                Constants.FRONT_LEFT_ANGLE_REVERSED, 
                Constants.FRONT_LEFT_STEER_OFFSET, 
                Constants.FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED, 
                Constants.FRONT_LEFT_ABSOLUTE_ENCODER_ID);
        private final SwerveModule frontRight = new SwerveModule(
                Constants.FRONT_RIGHT_DRIVE, 
                Constants.FRONT_RIGHT_STEER, 
                Constants.FRONT_RIGHT_MOTOR_REVERSED, 
                Constants.FRONT_RIGHT_ANGLE_REVERSED, 
                Constants.FRONT_RIGHT_STEER_OFFSET, 
                Constants.FRONT_RIGHT_ABSOULTE_ENCODER_REVERSED, 
                Constants.FRONT_RIGHT_ABSOLUTE_ENCODER_ID);
        private final SwerveModule backLeft = new SwerveModule(
                Constants.BACK_LEFT_DRIVE, 
                Constants.BACK_LEFT_STEER, 
                Constants.BACK_LEFT_MOTOR_REVERSED, 
                Constants.BACK_LEFT_ANGLE_REVERSED,
                Constants.BACK_LEFT_STEER_OFFSET, 
                Constants.BACK_LEFT_ABSOLUTE_ENCODER_REVERSED, 
                Constants.BACK_LEFT_ABSOLUTE_ENCODER_ID);
        private final SwerveModule backRight = new SwerveModule(
                Constants.BACK_RIGHT_DRIVE, 
                Constants.BACK_RIGHT_STEER, 
                Constants.BACK_RIGHT_MOTOR_REVERSED, 
                Constants.BACK_RIGHT_ANGLE_REVERSED, 
                Constants.BACK_RIGHT_STEER_OFFSET, 
                Constants.BACK_RIGHT_ABSOULTE_ENCODER_REVERSED, 
                Constants.BACK_RIGHT_ABSOLUTE_ENCODER_ID);
                private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics( // Creates an internal drawing of
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

        private final AHRS NavX = new AHRS(SPI.Port.kMXP);
        private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(Constants.FRONT_LEFT_DISTANCE_FROM_CENTER, Constants.FRONT_LEFT_ANGLE_FROM_CENTER),
                new SwerveModulePosition(Constants.FRONT_RIGHT_DISTANCE_FROM_CENTER, Constants.FRONT_RIGHT_ANGLE_FROM_CENTER),
                new SwerveModulePosition(Constants.BACK_LEFT_DISTANCE_FROM_CENTER, Constants.BACK_LEFT_ANGLE_FROM_CENTER),
                new SwerveModulePosition(Constants.BACK_RIGHT_DISTANCE_FROM_CENTER, Constants.BACK_RIGHT_ANGLE_FROM_CENTER)
        };
        private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0), modulePositions);

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        public void drive(ChassisSpeeds speeds){
                chassisSpeeds = speeds;
        }

        public void zeroHeading(){
                NavX.reset();
        }

        public double getHeading(){
                return Math.IEEEremainder(NavX.getAngle(), 360);
        }

        public Rotation2d getRotation2d(){
                        return Rotation2d.fromDegrees(getHeading());
        }

        public Rotation2d getGyroRotation(boolean fieldOrientation) { // Manually returns the gyro position as a

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
        public Pose2d getPose(){
                return odometer.getPoseMeters();
        }

        public void resetOdometry(Pose2d pose){
                odometer.resetPosition(getRotation2d(), modulePositions, pose);
        }

        public SwerveDriveKinematics getKinematics(){
                return kinematics;
        }

        @Override
        public void periodic() {
        odometer.update(getRotation2d(), modulePositions);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        }
        public void stopModules(){
                frontLeft.stop();
                frontRight.stop();
                backLeft.stop();
                backRight.stop();
        }

        public void setModuleStates(){
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED_METERS_PER_SECOND);

                frontLeft.setDesiredState(states[0]);
                frontRight.setDesiredState(states[1]);
                backLeft.setDesiredState(states[2]);
                backRight.setDesiredState(states[3]);
        }
}