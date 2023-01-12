package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wpiVcontainer.SwerveModule;

public class DriveTrain extends SubsystemBase {
  SwerveModule frontL = new SwerveModule(Constants.FRONT_LEFT_DRIVE_MOTOR,Constants.FRONT_LEFT_SPIN_MOTOR,Constants.FRONT_LEFT_CANCODER,Constants.FRONT_LEFT_DRIVE_ENCODER,Constants.FRONT_LEFT_CANCODER_OFFSET);
}
