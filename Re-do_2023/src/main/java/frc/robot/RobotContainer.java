// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.SusanHead;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Commands.ZeroSusan;
import frc.robot.Commands.ZeroVert;
import frc.robot.Commands.zeroHeading;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.VertArm;

public class RobotContainer {
  private final Joystick stickOne = new Joystick(0);
  private final Joystick stickTwo = new Joystick(1);
  private final Joystick stickThree = new Joystick(2);
  private final Joystick stickFour = new Joystick(3);

  private final Swerve swerve = new Swerve();
  private final LazySusanSub lazySusanSub = new LazySusanSub();
  private final ReachArmSub reachArmSub = new ReachArmSub();
  private final VertArm vertArm = new VertArm();
  private final Pneumatics pneumatics = new Pneumatics();
  private AutoContainer autoContainer = new AutoContainer();
  private HashMap<String, Command> eventMap;
  private SendableChooser<Integer> autoChooser;
  private SendableChooser<Integer> locationChooser;
  private SendableChooser<Integer> placementLevelChooser;
  private SendableChooser<Integer> placementSideChooser;
  private SendableChooser<Integer> balanceChooser;

  // private final SequentialCommandGroup aprilTagLeft = new SusanHead(lazySusanSub, 0)
  //     .andThen(new TagFollower(photonVision, mDriveTrain,
  //         Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT));

  // private final SequentialCommandGroup aprilTagRight = new SusanHead(lazySusanSub, 0)
  //     .andThen(new TagFollower(photonVision, mDriveTrain,
  //         Constants.WANTED_YAW_RIGHT, Constants.WANTED_SKEW_RIGHT, Constants.WANTED_DISTANCE_RIGHT));

  // private final SequentialCommandGroup aprilTagMid = new SusanHead(lazySusanSub, 0)
  //     .andThen(new TagFollower(photonVision, mDriveTrain,
  //         Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID, Constants.WANTED_DISTANCE_MID));

  public RobotContainer() {
    configureBindings();
    eventMap = new HashMap<>();
    autoChooser = new SendableChooser<>();
    locationChooser = new SendableChooser<>();
    placementLevelChooser = new SendableChooser<>();
    placementSideChooser = new SendableChooser<>();
    balanceChooser = new SendableChooser<>();

    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
    () -> modifyAxis(stickOne.getX()) * 0.3, 
    () -> modifyAxis(stickOne.getY()) * 0.3, 
    () -> modifyAxis(stickTwo.getX()) * 0.3, 
    () -> new JoystickButton(stickOne, 1).getAsBoolean()));
    

    reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () -> -stickFour.getRawAxis(1), 75));
    pneumatics.Pressurize();
    new zeroHeading(swerve);
    pneumatics.setDefaultCommand(new TogglePneumatics(pneumatics, false));
    vertArm.setDefaultCommand(
        new ZeroVert(vertArm).andThen(new ControlArm(vertArm, () -> modifyVertArm(stickThree.getRawAxis(1)), 30)));
    lazySusanSub.setDefaultCommand(
        new ZeroSusan(lazySusanSub).andThen(new ControlSusan(lazySusanSub, () -> modifyAxis(-stickFour.getX()), 50)));
  }

  private void configureBindings() {
    // april tags auto performance buttons
    // new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
    //     .whileTrue(aprilTagLeft);
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON)
    //     .whileTrue(aprilTagRight);
    // new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
    //     .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON))
    //     .whileTrue(aprilTagMid);

    // This lets a person press single button and open and close the claw every
    // other time.
    new JoystickButton(stickFour, Constants.TOGGLE_CLAW_BUTTON)
        .toggleOnTrue(new TogglePneumatics(pneumatics, !pneumatics.getChannel()));
      
    new JoystickButton(stickOne, 4).toggleOnFalse(new InstantCommand(() -> swerve.printModuleAngles()));


      new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnTrue(new zeroHeading(swerve));

    new JoystickButton(stickFour, Constants.SUSAN_ZERO_HEADING_BUTTON).onTrue(new SusanHead(lazySusanSub, 0));

    new JoystickButton(stickFour, Constants.SUSAN_TOGGLE_BUTTON).whileTrue(new ControlSusan(lazySusanSub, () -> - stickFour.getX(), 100));
    new JoystickButton(stickFour, Constants.SUSAN_TOGGLE_BUTTON).whileFalse(new ControlSusan(lazySusanSub, () -> stickFour.getX(), 100));



  }

  public void initializeAuto() {
    autoContainer = AutoContainer.getInstance();
      eventMap.put("InitializeValues", autoContainer.initializeCommand());
      eventMap.put("PrepElementPlacement", autoContainer.prepElementPlacementCommand());
      eventMap.put("PlaceElement", autoContainer.placeElementCommand());
      eventMap.put("PrepElementPickup", autoContainer.prepElementPickupCommand());
      eventMap.put("PickupElement", autoContainer.pickupElementCommand());

    autoContainer.autoInitialize(autoChooser, locationChooser, placementLevelChooser, placementSideChooser, balanceChooser, eventMap, swerve, pneumatics, vertArm, reachArmSub, lazySusanSub);
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(locationChooser);
    SmartDashboard.putData(placementLevelChooser);

  }

  public Command getAutonomousCommand() {
    return autoContainer.getAuto();
  }


  private static double modifyAxis(double value) {
    value = MathUtil.applyDeadband(value, 0.1);
    value = Math.copySign(value * value, value);
    return value;
  }

  /**
   * This is so that when the joystick does not have an output(or 0) it stays
   * still.
   */
  private double modifyVertArm(double value) {
    if (value < Constants.VERT_ARM_NO_DROP_SPEED && value > -0.075) {
      return Constants.VERT_ARM_NO_DROP_SPEED;
    } else {
      return value;
    } // changed to remove confusing math and limit for now [2-18 CP]
  }
}
