package frc.robot;

import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  private final AutoCommand m_autoCommand = new AutoCommand(driveTrain);

  public RobotContainer() {
  }

  public void initialize() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
