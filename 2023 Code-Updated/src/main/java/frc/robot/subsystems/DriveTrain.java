package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wpiVcontainer.SwerveModule;

public class DriveTrain extends SubsystemBase {
  SwerveModule frontL = new SwerveModule(1,3,4,2,8);

  public void Move(double angle, double speed){
    
  }
}