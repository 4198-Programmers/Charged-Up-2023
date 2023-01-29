package frc.robot;

import frc.robot.MathDriveFiles.MathDriveCom;
import frc.robot.MathDriveFiles.MathDriveTrain;
import frc.robot.ModuleBiasDrive.DriveTrainCom;
import frc.robot.ModuleBiasDrive.DriveTrainMod;
import frc.robot.commands.AutoMove;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.ControlArm;
import frc.robot.commands.ControlSusan;
import frc.robot.commands.MotorTestingCom;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopClaw;
import frc.robot.commands.StopSusan;
import frc.robot.subsystems.LazySusanSub;
import frc.robot.subsystems.MotorTesting;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VertArm;
import frc.robot.wpiDrive.FieldDriveCom;
import frc.robot.wpiDrive.FieldDriveSub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Subsystems
  // private final Pneumatics pneumaticsClaw = new Pneumatics();
  // private final MathDriveTrain mathDriveTrain = new MathDriveTrain();
  // private final LazySusanSub lazySusan = new LazySusanSub();
  // private final VertArm vertArm = new VertArm();
  // private final FieldDriveSub fieldDriveTrain = new FieldDriveSub();
  // MotorTesting testing = new MotorTesting();
  DriveTrainMod driveTrainModular = new DriveTrainMod();

  private final Joystick joystickLeft = new Joystick(0); // Joysticks
  private final Joystick joystickMid = new Joystick(1);
  private final Joystick joystickRight = new Joystick(2);
  private final Trigger clawBTN = new JoystickButton(joystickLeft, 1);

  // private final FieldDriveCom fieldDriving = new FieldDriveCom(fieldDriveTrain,
  // joystickLeft, joystickMid);
  // private final MathDriveCom mathDriving = new MathDriveCom(mathDriveTrain,
  // joystickLeft, joystickMid);
  // private final StopClaw stopClaw = new StopClaw(pneumaticsClaw);
  // private final OpenClaw openClaw = new OpenClaw(pneumaticsClaw);
  // MotorTestingCom testCom = new MotorTestingCom(testing, joystickLeft);

  // All the comments are pre-made code that can't be initialized yet

  // private final CloseClaw closeClaw = new CloseClaw(pneumaticsClaw);
  // private final ControlSusan controlSusan = new ControlSusan(lazySusan,
  // joystickRight);
  // private final StopSusan stopSusan = new StopSusan(lazySusan);
  // private final ControlArm controlArm = new ControlArm(vertArm, joystickRight);
  // private final StopArm stopArm = new StopArm(vertArm);

  public RobotContainer() {
    driveTrainModular.setDefaultCommand(new DriveTrainCom(driveTrainModular,
        () -> -modifyAxis(joystickLeft.getY()) * DriveTrainMod.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystickLeft.getX()) * DriveTrainMod.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystickMid.getX()) * DriveTrainMod.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
  }

  public void initialize() {
    configureButtonBindings();
    // testing.setDefaultCommand(testCom);
    // fieldDriveTrain.reset();
    // fieldDriveTrain.setDefaultCommand(fieldDriving); //to change driving modes
    // simply switch which is commented out
    // mathDriveTrain.setDefaultCommand(mathDriving);
    // vertArm.setDefaultCommand(controlArm);
    // lazySusan.setDefaultCommand(controlSusan);
    // pneumaticsClaw.setDefaultCommand(stopClaw);

  }

  public void zeroGyro(){
    driveTrainModular.zeroGyro();
    // fieldDriveTrain.resetGyro();
  }

  private static double deadband(double value, double deadband) { // for controller drift, keeps values between 0-1
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.1);
    value = Math.copySign(value * value, value);
    return value;
  }

  public void updating() {
    // fieldDriveTrain.displayGyro();
    driveTrainModular.getPrint();
  }

  private void configureButtonBindings() {
    // clawBTN.toggleOnTrue(closeClaw);
    // clawBTN.toggleOnFalse(openClaw);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
