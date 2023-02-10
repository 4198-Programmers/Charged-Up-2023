// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.CloseClaw;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlClaw;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Commands.OpenClaw;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

public class RobotContainer {
  private final Joystick stickOne = new Joystick(0);
  private final Joystick stickTwo = new Joystick(1);
  private final Joystick stickThree = new Joystick(2);

  private final DriveTrain mDriveTrain = new DriveTrain();
  private final LazySusanSub lazySusanSub = new LazySusanSub();
  private final ReachArmSub reachArmSub = new ReachArmSub();
  private final VertArm vertArm = new VertArm();  
  private final Pneumatics pneumatics = new Pneumatics();
  private final ControlSusan controlSusan = new ControlSusan(lazySusanSub, ()-> (-stickThree.getRawAxis(0)), 100);
  private final ControlReach reachOut = new ControlReach(reachArmSub, () -> 1);
  private final ControlReach reachIn = new ControlReach(reachArmSub, () ->-1);
  

  private final ControlArm controlArm = new ControlArm(vertArm, () -> stickThree.getRawAxis(1), 100);

  public RobotContainer() {
    configureBindings();
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    lazySusanSub.setDefaultCommand(controlSusan);
    vertArm.setDefaultCommand(controlArm);
  }
  //int reachOutButton = stickThree.getPOV();
  private void configureBindings() {
    new POVButton(stickThree, 0).onTrue(reachOut);
    new POVButton(stickThree, 180).onTrue(reachIn);
    new JoystickButton(stickThree, Constants.REACH_OUT_BUTTON).onTrue(reachOut);
    new JoystickButton(stickThree, Constants.REACH_IN_BUTTON).onTrue(reachIn);
    new JoystickButton(stickThree, Constants.CLAW_OPEN_BUTTON).onTrue(new OpenClaw(pneumatics));
    new JoystickButton(stickThree, Constants.CLAW_CLOSE_BUTTON).onTrue(new CloseClaw(pneumatics));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static double deadband(double value, double deadband) {
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
    // Deadband
    value = deadband(value, 0.1);
    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void resetGyro() {
    mDriveTrain.zeroGyro();
  }
}
