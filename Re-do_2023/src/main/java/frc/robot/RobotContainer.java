// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.CloseClaw;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlClaw;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Commands.OpenClaw;
import frc.robot.Commands.zeroHeading;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

public class RobotContainer {
  private final Joystick stickOne = new Joystick(0);
  private final Joystick stickTwo = new Joystick(1);
  private final Joystick stickThree = new Joystick(2);
  private final Joystick stickFour = new Joystick(3);
  
  // private final PhotonVision photonVision = new PhotonVision();
  private final DriveTrain mDriveTrain = new DriveTrain();
  private final LazySusanSub lazySusanSub = new LazySusanSub();
  private final ReachArmSub reachArmSub = new ReachArmSub();
  private final VertArm vertArm = new VertArm();  
  private final Pneumatics pneumatics = new Pneumatics();
  private final ControlSusan controlSusan = new ControlSusan(lazySusanSub, ()-> (-stickFour.getRawAxis(0)), 100);
  private final ControlReach reach = new ControlReach(reachArmSub, () -> stickThree.getRawAxis(1));
  

  public RobotContainer() {
    configureBindings();
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .1,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .1,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .1)); //ATTENTION These values were multiplied by Oren to make the bot not die while testing the three  * .1 terms should be deleted

    lazySusanSub.setDefaultCommand(controlSusan);
    reachArmSub.setDefaultCommand(reach);
    pneumatics.Pressurize();
    mDriveTrain.zeroGyro();


  }
  //int reachOutButton = stickThree.getPOV();
  private void configureBindings() {
    new POVButton(stickThree, -1).onTrue(new ControlReach(reachArmSub, () -> 0));

    new JoystickButton(stickFour, 1).onTrue(new ControlArm(vertArm, () -> -stickFour.getRawAxis(1), 100));
    new JoystickButton(stickFour, 1).onFalse(new ControlArm(vertArm, () -> 0, 100));
    
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_LEFT_BUTTON)
    //   .whileTrue(new TagFollower(photonVision, mDriveTrain, Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT));
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON)
    //   .whileTrue(new TagFollower(photonVision, mDriveTrain, Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID, Constants.WANTED_DISTANCE_MID));
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_CENTER_BUTTON)
    //   .whileTrue(new TagFollower(photonVision, mDriveTrain, Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID, Constants.WANTED_DISTANCE_MID));

    new JoystickButton(stickThree, Constants.ON_TRIGGER_CLAW_BUTTON).onTrue(new ControlClaw(pneumatics));
    new JoystickButton(stickThree, 4).onTrue(new OpenClaw(pneumatics));
    new JoystickButton(stickFour, 3).onTrue(new CloseClaw(pneumatics));
    new JoystickButton(stickOne, 11).onTrue(new zeroHeading(mDriveTrain));
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
