package frc.robot;

import frc.robot.commands.AutoCommand;
import frc.robot.commands.CenterTag;
import frc.robot.commands.CheckPhotonTarget;
import frc.robot.commands.DistanceTag;
import frc.robot.commands.FlattenTag;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  private final PhotonVision photonVision = new PhotonVision();
  private final AutoCommand m_autoCommand = new AutoCommand(driveTrain);
  private final Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK);
  private final Joystick midJoystick = new Joystick(Constants.MID_JOYSTICK);
  private final Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK);
  private final Trigger button1 = new JoystickButton(leftJoystick, 1);
  private final Trigger button2 = new JoystickButton(leftJoystick, 2);
  
  
  Command setAprilTagPosition = (new FlattenTag(photonVision, driveTrain))
      .andThen(new CenterTag(photonVision, driveTrain))
      .andThen(new DistanceTag(photonVision, driveTrain));

  public RobotContainer() {
    
  }

  public void initialize() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    button1.whileTrue(new CheckPhotonTarget(photonVision));    
    button2.whileTrue(setAprilTagPosition);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
