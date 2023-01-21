package frc.robot;

import frc.robot.commands.AutoCommand;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.MathDriveCom;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.StopClaw;
import frc.robot.subsystems.MathDriveTrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.wpiVcontainer.FieldDriveCom;
import frc.robot.wpiVcontainer.FieldDriveSub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  //private final FieldDriveSub fieldDriveTrain = new FieldDriveSub(); // Subsystems
  private final Pneumatics pneumaticsClaw = new Pneumatics();
  private final MathDriveTrain mathDriveTrain = new MathDriveTrain();

  private final Joystick joystickLeft = new Joystick(0); // Joysticks
  private final Joystick joystickMid = new Joystick(1);
  private final Trigger clawBTN = new JoystickButton(joystickLeft, 1);

  //private final FieldDriveCom fieldDriving = new FieldDriveCom(fieldDriveTrain, joystickLeft, joystickMid);
  private final MathDriveCom mathDriving = new MathDriveCom(mathDriveTrain, joystickLeft, joystickMid);
  private final StopClaw stopClaw = new StopClaw(pneumaticsClaw);
  private final OpenClaw openClaw = new OpenClaw(pneumaticsClaw);
  private final CloseClaw closeClaw = new CloseClaw(pneumaticsClaw);

  public RobotContainer() {
  }

  public void initialize() {
    configureButtonBindings();
    // fieldDriveTrain.setDefaultCommand(fieldDriving); //to change driving modes
    // simply switch which is commented out
    mathDriveTrain.setDefaultCommand(mathDriving);
    pneumaticsClaw.setDefaultCommand(stopClaw);

  }

  private void configureButtonBindings() {
    clawBTN.toggleOnTrue(closeClaw);
    clawBTN.toggleOnFalse(openClaw);
  }

  public Command getAutonomousCommand() {
    return stopClaw;
  }
}
