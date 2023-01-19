package frc.robot;

import frc.robot.commands.AutoCommand;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.StopClaw;
import frc.robot.subsystems.Pneumatics;
import frc.robot.wpiVcontainer.DriveCommand;
import frc.robot.wpiVcontainer.DriveTrainW;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private boolean pressed = false;

  private final DriveTrainW driveTrain = new DriveTrainW();
  private final Pneumatics pneumaticsClaw = new Pneumatics();

  private final Joystick joystick = new Joystick(0);
  private final Trigger clawBTN = new JoystickButton(joystick, 1);

  private final AutoCommand m_autoCommand = new AutoCommand(driveTrain);
  private final DriveCommand driving = new DriveCommand(driveTrain, joystick);
  private final StopClaw stopClaw = new StopClaw(pneumaticsClaw);
  private final OpenClaw openClaw = new OpenClaw(pneumaticsClaw);
  private final CloseClaw closeClaw = new CloseClaw(pneumaticsClaw);
  
  public RobotContainer() {
  }

  public void initialize() {
    configureButtonBindings();
    driveTrain.setDefaultCommand(driving);
    pneumaticsClaw.setDefaultCommand(stopClaw);
  }

  private void configureButtonBindings() {
    clawBTN.toggleOnTrue(closeClaw);
    clawBTN.toggleOnFalse(openClaw);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
