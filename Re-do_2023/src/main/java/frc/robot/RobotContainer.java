// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Autos;
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
import frc.robot.Commands.Autos.AutoTypes;
import frc.robot.Commands.Autos.Locations;
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
  private Autos autos = new Autos();
  private HashMap<String, Command> eventMap;
  private SendableChooser<AutoTypes> autoChooser;
  private SendableChooser<Locations> locationChooser;

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

    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
    () -> stickOne.getX() * 0.3, 
    () -> stickOne.getY() * 0.3, 
    () -> stickTwo.getX() * 0.3, 
    () -> new JoystickButton(stickOne, 1).getAsBoolean()));
    

    reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () -> -stickFour.getRawAxis(1), 75));
    pneumatics.Pressurize();
    new zeroHeading(swerve);
    pneumatics.setDefaultCommand(new TogglePneumatics(pneumatics, false));
    vertArm.setDefaultCommand(
        new ZeroVert(vertArm).andThen(new ControlArm(vertArm, () -> modifyVertArm(stickThree.getRawAxis(1)), 30)));
    lazySusanSub.setDefaultCommand(
        new ZeroSusan(lazySusanSub).andThen(new ControlSusan(lazySusanSub, () -> modifyAxis(-stickFour.getX()), 50)));
    lazySusanSub.mode(IdleMode.kBrake);
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


      new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnTrue(new zeroHeading(swerve));

    new JoystickButton(stickFour, Constants.SUSAN_ZERO_HEADING_BUTTON).onTrue(new SusanHead(lazySusanSub, 0));

    new JoystickButton(stickFour, Constants.SUSAN_TOGGLE_BUTTON).whileTrue(new ControlSusan(lazySusanSub, () -> - stickFour.getX(), 100));
    new JoystickButton(stickFour, Constants.SUSAN_TOGGLE_BUTTON).whileFalse(new ControlSusan(lazySusanSub, () -> stickFour.getX(), 100));



  }

  public void initializeAuto() {
    autos = Autos.getInstance();
    // eventMap.put("PrepElementPlacement", new SequentialCommandGroup(new InstantCommand(() -> pneumatics.togglePneumatics(false), pneumatics), 
    //   new InstantCommand(() -> vertArm.autoVert(0.5, Constants.MAX_VERTICAL_POSITION), vertArm)));

    // eventMap.put("PlaceElement", new SequentialCommandGroup(
    //   new InstantCommand(() -> reachArmSub.moveReach(0.5), reachArmSub).raceWith(new WaitCommand(1)),
    //   new InstantCommand(() -> pneumatics.togglePneumatics(true), pneumatics),
    //   new InstantCommand(() -> pneumatics.togglePneumatics(false), pneumatics),
    //   new InstantCommand(() -> reachArmSub.moveReach(0.5), reachArmSub).raceWith(new WaitCommand(1)),
    //   new InstantCommand(() -> vertArm.autoVert(-0.5, Constants.MIN_VERTICAL_POSITION), vertArm)
    // ));

    // eventMap.put("PrepElementPickup", 
    // new SequentialCommandGroup(
    //   new InstantCommand(() -> vertArm.autoVert(0.5, Constants.VERT_PICKUP_POS), vertArm),
    //   new InstantCommand(() -> pneumatics.togglePneumatics(true), pneumatics)
    // ));

    // eventMap.put("PickupElement", 
    // new SequentialCommandGroup(
    //   new InstantCommand(() -> pneumatics.togglePneumatics(true))
    // ));
      eventMap.put("PrepElementPlacement", new PrintCommand("Prep Element Placement"));
      eventMap.put("PlaceElement", new PrintCommand("Place Element"));
      eventMap.put("PrepElementPickup", new PrintCommand("Prep Element Pickup"));
      eventMap.put("PickupElement", new PrintCommand("Pickup Element"));

    autos.autoInitialize(autoChooser, locationChooser, eventMap, swerve);
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(locationChooser);

  }

  public Command getAutonomousCommand() {
    return autos.getAuto();
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
