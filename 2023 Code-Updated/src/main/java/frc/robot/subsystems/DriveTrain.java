package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wpiVcontainer.SwerveModule;

public class DriveTrain extends SubsystemBase {
  SwerveModule frontR = new SwerveModule(Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.FRONT_RIGHT_SPIN_MOTOR_ID,
      Constants.FRONT_RIGHT_CANCODER_ID, Constants.FRONT_RIGHT_DRIVE_ENCODER_ID, Constants.FRONT_RIGHT_CANCODER_OFFSET);

  SwerveModule frontL = new SwerveModule(Constants.FRONT_LEFT_DRIVE_MOTOR_ID, Constants.FRONT_LEFT_SPIN_MOTOR_ID,
      Constants.FRONT_LEFT_CANCODER_ID, Constants.FRONT_LEFT_DRIVE_ENCODER_ID, Constants.FRONT_LEFT_CANCODER_OFFSET);

  SwerveModule backR = new SwerveModule(Constants.BACK_LEFT_DRIVE_MOTOR_ID, Constants.BACK_LEFT_SPIN_MOTOR_ID,
      Constants.BACK_LEFT_CANCODER_ID, Constants.BACK_LEFT_DRIVE_ENCODER_ID, Constants.BACK_LEFT_CANCODER_OFFSET);

  SwerveModule backL = new SwerveModule(Constants.BACK_LEFT_DRIVE_MOTOR_ID, Constants.BACK_LEFT_SPIN_MOTOR_ID,
      Constants.BACK_LEFT_CANCODER_ID, Constants.BACK_LEFT_DRIVE_ENCODER_ID, Constants.BACK_LEFT_CANCODER_OFFSET);
}
