package frc.robot.AttemptThree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// public class DriveTrainSubsystem extends SubsystemBase{
//     private final SwerveModule frontLeftModule;
//     private final SwerveModule frontRightModule;
//     private final SwerveModule backLeftModule;
//     private final SwerveModule backRightModule;

//     public double xSpeed;
//     public double ySpeed;
//     public double zSpeed;

//     private static final SwerveDriveKinematics kinematics = Constants.SWERVE_DRIVE_KINEMATICS;

//     public DriveTrainSubsystem(){
//         frontLeftModule = new SwerveModule(
//             Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
//             Constants.FRONT_LEFT_ANGLE_MOTOR_ID, 
//             Constants.FRONT_LEFT_CANCODER_ID, 
//             new Rotation2d(Constants.FRONT_LEFT_ANGLE_OFFSET));
//         frontRightModule = new SwerveModule(
//             Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
//             Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
//             Constants.FRONT_RIGHT_CANCODER_ID, 
//             new Rotation2d(Constants.FRONT_RIGHT_ANGLE_OFFSET));
//         backLeftModule = new SwerveModule(
//             Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
//             Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
//             Constants.BACK_LEFT_CANCODER_ID, 
//             new Rotation2d(Constants.BACK_LEFT_ANGLE_OFFSET));
//         backRightModule = new SwerveModule(
//             Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
//             Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
//             Constants.BACK_RIGHT_CANCODER_ID, 
//             new Rotation2d(Constants.BACK_RIGHT_ANGLE_OFFSET));
//     }

//     public void drive(ChassisSpeeds chassisSpeeds){
//         SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
//         setModuleStates(swerveModuleStates);
//     }
//     public void drive(double Xspeed, double Yspeed, double Zspeed){
//         Xspeed = Math.copySign(Math.min(Math.abs(Xspeed), 1.0), Xspeed);
//         Yspeed = Math.copySign(Math.min(Math.abs(Yspeed), 1.0), Yspeed);
//         Zspeed = Math.copySign(Math.min(Math.abs(Zspeed), 1.0), Zspeed);
//         ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
//             Xspeed,
//             ySpeed,
//             zSpeed
//         );
//     }
// }
