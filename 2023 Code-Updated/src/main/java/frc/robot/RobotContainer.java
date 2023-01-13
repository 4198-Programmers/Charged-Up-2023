package frc.robot;

import frc.robot.commands.AutoCommand;
import frc.robot.commands.MotorTesting;
import frc.robot.subsystems.DriveTrain;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  private final AutoCommand m_autoCommand = new AutoCommand(driveTrain);
  Joystick joystick = new Joystick(0);
  Trigger motorTester = new JoystickButton(joystick, 11);
  MotorTesting motorTesting = new MotorTesting(joystick, 2, MotorType.kBrushless);

  public RobotContainer() {
  }

  public void initialize() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    motorTester.whileTrue(motorTesting);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
