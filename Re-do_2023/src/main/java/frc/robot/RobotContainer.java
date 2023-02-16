// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.CloseClaw;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Commands.SusanMode;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Commands.ToggleChannels;
import frc.robot.Commands.zeroHeading;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

//All PhotonVision subsystems/commands had to be commented out because the camera does not exist yet.

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
  

  public RobotContainer() {
    configureBindings();
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5, true)); //ATTENTION These values were multiplied by Oren to make the bot not die while testing the three  * .5 terms should be deleted

    reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () ->-stickFour.getRawAxis(1)));
    pneumatics.Pressurize();
    new zeroHeading(mDriveTrain, true); //This sets the robot front to be the forward direction
    pneumatics.setDefaultCommand(new CloseClaw(pneumatics));
    vertArm.setDefaultCommand(new ControlArm(vertArm, () -> modifyVertArm(stickThree.getRawAxis(1)), 100));
    lazySusanSub.mode(IdleMode.kBrake);
  }
  private void configureBindings() {
    
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_LEFT_BUTTON)
    //   .whileTrue(new TagFollower(photonVision, mDriveTrain, Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT));
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON)
    //   .whileTrue(new TagFollower(photonVision, mDriveTrain, Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID, Constants.WANTED_DISTANCE_MID));
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_CENTER_BUTTON)
    //   .whileTrue(new TagFollower(photonVision, mDriveTrain, Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID, Constants.WANTED_DISTANCE_MID));

//This lets a person press single button and open and close the claw every other time.
    new JoystickButton(stickFour, Constants.TOGGLE_CLAW_BUTTON).toggleOnTrue(new ToggleChannels(pneumatics, !pneumatics.getChannel()));

//This resets the robot to field orientation and sets the current front of the robot to the forward direction
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).onTrue(new zeroHeading(mDriveTrain, true));
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).onTrue(new DriveTrainCom(
      mDriveTrain,
      () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
      () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
      () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5, true));

      new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnFalse(new zeroHeading(mDriveTrain, true));
      new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnFalse(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5, true));

      // This makes the front of the robot always the forward direction.
      //When fieldOrientation is false, it is in robotOrientation.
    new JoystickButton(stickOne, Constants.ROBOT_ORIENTATION_BUTTON).onTrue(new DriveTrainCom(
      mDriveTrain,
      () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
      () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
      () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5, false));
    new JoystickButton(stickOne, Constants.ROBOT_ORIENTATION_BUTTON).toggleOnFalse(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5, false));

//Make sure susan is set to a low value because it spins really fast. It has to be at least under 0.3, most likely.
    new JoystickButton(stickFour, Constants.LAZY_SUSAN_LEFT_BUTTON).onTrue(new ControlSusan(lazySusanSub, () -> 0.1, 100));
    new JoystickButton(stickFour, Constants.LAZY_SUSAN_RIGHT_BUTTON).onTrue(new ControlSusan(lazySusanSub, () -> -0.1, 100));
    new JoystickButton(stickFour, Constants.SUSAN_BRAKE_BUTTON).onTrue(new SusanMode(lazySusanSub, IdleMode.kBrake));
    new JoystickButton(stickFour, Constants.SUSAN_COAST_BUTTON).onTrue(new SusanMode(lazySusanSub, IdleMode.kCoast));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  /*This and the modifyAxis methods are used so that when the joystick isn't being used, that
   * the speed is set to 0. This is incase the joysticks don't rest at 0 exactly.
   */
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
  /**This is so that when the joystick does not have an output(or 0) it stays still. */
  private double modifyVertArm(double value){
    if(vertArm.getLocation() <=0.46){
      return 0.25*value;
    }
    return (0.25*value) + 3*0.03125;
  }

  public void resetGyro() {
    mDriveTrain.zeroGyro();
  }
}
