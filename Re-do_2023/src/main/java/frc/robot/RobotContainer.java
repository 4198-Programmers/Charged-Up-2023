// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Commands.RunPathAuto;
import frc.robot.Commands.SusanHead;
import frc.robot.Commands.SusanMode;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.PathHolder;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Commands.ToggleSusan;
import frc.robot.Commands.WithMarker;
import frc.robot.Commands.ZeroSusan;
import frc.robot.Commands.ZeroVert;
import frc.robot.Commands.zeroHeading;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;
import frc.robot.Subsystems.PathHolder.PathChoice;

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
  private final PathHolder mPath = new PathHolder(vertArm, pneumatics, reachArmSub, lazySusanSub);

  // private AutoContainer mAutoContainer = new AutoContainer(mDriveTrain,
  // lazySusanSub, pneumatics, reachArmSub, vertArm);
  // private final SendableChooser<Location> LocationChooser = new
  // SendableChooser<>();
  // private final SendableChooser<AutoType> AutoChooser = new
  // SendableChooser<>();
  // private final SendableChooser<LevelPriority> LevelChooser = new
  // SendableChooser<>();
  private final RunPathAuto autoPath = new RunPathAuto(mPath, mDriveTrain);
  private final SendableChooser<PathChoice> PathChooser = new SendableChooser<>();

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
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5,
        true)); // ATTENTION These values were multiplied by Oren to make the bot not die while
                // testing the three * .5 terms should be deleted

    reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () -> -stickFour.getRawAxis(1), 75));
    pneumatics.Pressurize();
    new zeroHeading(mDriveTrain); // This sets the robot front to be the forward direction
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

        new JoystickButton(stickFour, Constants.TOGGLE_SUSAN_DIRECTION_BUTTON).toggleOnTrue(new ToggleSusan(lazySusanSub));

    // This resets the robot to field orientation and sets the current front of the
    // robot to the forward direction
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).onTrue(new zeroHeading(mDriveTrain));
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).onTrue(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5,
        true));

    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnFalse(new zeroHeading(mDriveTrain));
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnFalse(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5,
        true));

    // This makes the front of the robot always the forward direction.
    // When fieldOrientation is false, it is in robotOrientation.
    new JoystickButton(stickOne, Constants.ROBOT_ORIENTATION_BUTTON).onTrue(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5,
        false));
    new JoystickButton(stickOne, Constants.ROBOT_ORIENTATION_BUTTON).toggleOnFalse(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5,
        false));

    // Make sure susan is set to a low value because it spins really fast. It has to
    // be at least under 0.3, most likely.
    new JoystickButton(stickFour, Constants.SUSAN_ZERO_HEADING_BUTTON).onTrue(new SusanHead(lazySusanSub, 0));
    // Make sure susan is set to a low value because it spins really fast. It has to
    // be at least under 0.3, most likely. -> That is what the % modifier is for.
    // Don't change the speed -CP
    // new JoystickButton(stickFour, Constants.LAZY_SUSAN_LEFT_BUTTON)
    // .whileTrue(new ControlSusan(lazySusanSub, () -> 1, 10));
    // new JoystickButton(stickFour, Constants.LAZY_SUSAN_RIGHT_BUTTON)
    // .whileTrue(new ControlSusan(lazySusanSub, () -> -1, 10));

    new JoystickButton(stickOne, 1).whileTrue(new Balance(mDriveTrain));
  }

  public void initializeAuto() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Choices");
    autoTab.add(PathChooser);
    // autoTab.add(AutoChooser);
    // autoTab.add(LocationChooser);
    // autoTab.add(LevelChooser);
    // LocationChooser.setDefaultOption("Left", Location.Left);
    // LocationChooser.addOption("Middle", Location.Middle);
    // LocationChooser.addOption("Right", Location.Right);
    // AutoChooser.addOption("One Element, No Balance",
    // AutoType.OneElementNoBalance);
    // AutoChooser.addOption("Two Element, No Balance",
    // AutoType.TwoElementNoBalance);
    // AutoChooser.addOption("One Element, Balance", AutoType.OneElementBalance);
    // AutoChooser.setDefaultOption("Two Element, Balance",
    // AutoType.TwoElementBalance);
    // AutoChooser.addOption("Three Element, Balance",
    // AutoType.ThreeElementBalance);
    // LevelChooser.setDefaultOption("Floor", LevelPriority.Floor);
    // LevelChooser.addOption("Middle", LevelPriority.Mid);
    // LevelChooser.addOption("Top", LevelPriority.Top);

    PathChooser.setDefaultOption("Left One Element No Balance", PathChoice.LeftOneElement);
    PathChooser.addOption("Left Three Element Balance", PathChoice.LeftTwoElement);
    PathChooser.addOption("Left Three Element Balance", PathChoice.LeftOneElementBalance);
    PathChooser.addOption("Left Three Element Balance", PathChoice.LeftTwoElementBalance);
    PathChooser.addOption("Left Three Element Balance", PathChoice.LeftThreeElementBalance);
    PathChooser.addOption("Mid Three Element Balance", PathChoice.MiddleOneElement);
    PathChooser.addOption("Mid Three Element Balance", PathChoice.MiddleTwoElement);
    PathChooser.addOption("Mid Three Element Balance", PathChoice.MiddleOneElementBalance);
    PathChooser.addOption("Mid Three Element Balance", PathChoice.MiddleTwoElementBalance);
    PathChooser.addOption("Mid Three Element Balance", PathChoice.MiddleThreeElementBalance);
    PathChooser.addOption("Right Three Element Balance", PathChoice.RightOneElement);
    PathChooser.addOption("Right Three Element Balance", PathChoice.RightTwoElement);
    PathChooser.addOption("Right Three Element Balance", PathChoice.RightOneElementBalance);
    PathChooser.addOption("Right Three Element Balance", PathChoice.RightTwoElementBalance);
    PathChooser.addOption("Right Three Element Balance", PathChoice.RightThreeElementBalance);
    PathChooser.addOption("Drive Straight", PathChoice.DriveStraight);
    

  }

  public Command getAutonomousCommand() {
    // return mAutoContainer.autoRunCommand();
    // return Commands.print("No autonomous command configured");
    // return autoPath;
    return new RunPathAuto(mPath, mDriveTrain);
    // return new WithMarker(mDriveTrain, mPath);
  }

  /*
   * This and the modifyAxis methods are used so that when the joystick isn't
   * being used, that
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
