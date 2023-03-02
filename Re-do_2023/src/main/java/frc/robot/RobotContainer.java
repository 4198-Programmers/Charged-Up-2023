// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Commands.TagFollower;
import frc.robot.Commands.SlightTurnDrive;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.PathHolder;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Commands.ToggleSusan;
import frc.robot.Commands.ZeroSusan;
import frc.robot.Commands.ZeroVert;
import frc.robot.Commands.zeroHeading;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.SinglePaths;
import frc.robot.Subsystems.VertArm;
import frc.robot.Subsystems.SinglePaths.BalanceSP;
import frc.robot.Subsystems.SinglePaths.Elements;
// import frc.robot.Subsystems.PathHolder.PathChoice;
import frc.robot.Subsystems.SinglePaths.Location;

public class RobotContainer {
  private final Joystick stickOne = new Joystick(0);
  private final Joystick stickTwo = new Joystick(1);
  private final Joystick stickThree = new Joystick(2);
  private final Joystick stickFour = new Joystick(3);

  private final PhotonVision photonVision = new PhotonVision();
  private final DriveTrain mDriveTrain = new DriveTrain();
  private final LazySusanSub lazySusanSub = new LazySusanSub();
  private final ReachArmSub reachArmSub = new ReachArmSub();
  private final VertArm vertArm = new VertArm();
  private final Pneumatics pneumatics = new Pneumatics();

  // april tag commands
  private final Command aprilTagLeft = new AutoSusan(lazySusanSub, 0.25, 0).andThen(
      new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT));

  private final Command aprilTagLeftTopLeft = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0));

  private final Command aprilTagLeftTopRight = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25,
          Maths.degreesToRotations_Susan(Math.toDegrees(Math.tanh((Constants.CUBE_TO_CONE_NODE_WIDTH)
              / (Constants.CAMERA_TO_DRIVEBASE_OFFSET + Constants.END_OF_BOT_TO_TOP_NODE_LENGTH))))));

  private final Command aprilTagLeftMidLeft = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0));

  private final Command aprilTagLeftMidRight = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25,
          Maths.degreesToRotations_Susan(Math.toDegrees(Math.tanh((Constants.CUBE_TO_CONE_NODE_WIDTH)
              / (Constants.CAMERA_TO_DRIVEBASE_OFFSET + Constants.END_OF_BOT_TO_MID_NODE_LENGTH))))));

  private final Command aprilTagRight = new AutoSusan(lazySusanSub, 0.25, 0).andThen(
      new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_RIGHT, Constants.WANTED_SKEW_RIGHT, Constants.WANTED_DISTANCE_RIGHT));

  private final Command aprilTagRightTopLeft = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0));

  private final Command aprilTagRightTopRight = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25,
          Maths.degreesToRotations_Susan(Math.toDegrees(Math.tanh((Constants.CUBE_TO_CONE_NODE_WIDTH)
              / (Constants.CAMERA_TO_DRIVEBASE_OFFSET + Constants.END_OF_BOT_TO_TOP_NODE_LENGTH))))));

  private final Command aprilTagRightMidLeft = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0));

  private final Command aprilTagRightMidRight = new AutoVert(vertArm, 0.25, Constants.LEVEL_ARM)
      .andThen(new AutoSusan(lazySusanSub, 0.25, 0))
      .andThen(new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT, Constants.WANTED_DISTANCE_LEFT))
      .andThen(new AutoSusan(lazySusanSub, 0.25,
          Maths.degreesToRotations_Susan(Math.toDegrees(Math.tanh((Constants.CUBE_TO_CONE_NODE_WIDTH)
              / (Constants.CAMERA_TO_DRIVEBASE_OFFSET + Constants.END_OF_BOT_TO_MID_NODE_LENGTH))))));

  private final Command aprilTagMid = new AutoSusan(lazySusanSub, 0.25, 0).andThen(
      new TagFollower(photonVision, mDriveTrain,
          Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID, Constants.WANTED_DISTANCE_MID));

  private final PathHolder mPath = new PathHolder(vertArm, pneumatics, reachArmSub, lazySusanSub);
  private final SinglePaths singlePaths = new SinglePaths(mDriveTrain, vertArm, lazySusanSub, pneumatics);

  // private AutoContainer mAutoContainer = new AutoContainer(mDriveTrain,
  // lazySusanSub, pneumatics, reachArmSub, vertArm);
  // private final SendableChooser<Location> LocationChooser = new
  // SendableChooser<>();
  // private final SendableChooser<AutoType> AutoChooser = new
  // SendableChooser<>();
  // private final SendableChooser<LevelPriority> LevelChooser = new
  // SendableChooser<>();
  private final SendableChooser<String> PathChooser = new SendableChooser<>();
  private final SendableChooser<Integer> LocationChooser = new SendableChooser<>();
  private final SendableChooser<Integer> ElementsChooser = new SendableChooser<>();
  private final SendableChooser<Integer> BalanceChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    pipeline.setDouble(0);
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * .5,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5,
        true)); // ATTENTION These values were multiplied by Oren to make the bot not die while
                // testing the three * .5 terms should be deleted

    // reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () ->
    // -stickThree.getRawAxis(1), 75)); //CHANGETOTHREE
    reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () -> 0, 0));
    pneumatics.Pressurize();
    new zeroHeading(mDriveTrain); // This sets the robot front to be the forward direction
    pneumatics.setDefaultCommand(new TogglePneumatics(pneumatics, false));
    vertArm.setDefaultCommand(new ControlArm(vertArm, () -> modifyVertArm(stickThree.getRawAxis(1)), 30));
    lazySusanSub.setDefaultCommand(
        new ControlSusan(lazySusanSub, () -> modifyAxis(-stickThree.getX()), 30));// CHANGETOTHREE
    lazySusanSub.mode(IdleMode.kBrake);
  }

  public void initShuffleboard() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Choices");
    autoTab.add("Autonomous", PathChooser);
    PathChooser.setDefaultOption("Left One Element No Balance", "LeftOneElement");
    PathChooser.addOption("Left Two Element No Balance", "LeftTwoElement");
    PathChooser.addOption("Left One Element Balance", "LeftOneElementBalance");
    PathChooser.addOption("Left Two Element Balance", "LeftTwoElementBalance");
    PathChooser.addOption("Left Three Element Balance", "LeftThreeElementBalance");
    PathChooser.addOption("Mid One Element No Balance", "MiddleOneElement");
    PathChooser.addOption("Mid Two Element No Balance", "MiddleTwoElement");
    PathChooser.addOption("Mid One Element Balance", "MiddleOneElementBalance");
    PathChooser.addOption("Mid Two Element Balance", "MiddleTwoElementBalance");
    PathChooser.addOption("Mid Three Element Balance", "MiddleThreeElementBalance");
    PathChooser.addOption("Right One Element No Balance", "RightOneElement");
    PathChooser.addOption("Right Two Element No Balance", "RightTwoElement");
    PathChooser.addOption("Right One Element No Balance", "RightOneElementBalance");
    PathChooser.addOption("Right Two Element No Balance", "RightTwoElementBalance");
    PathChooser.addOption("Right Three Element No Balance", "RightThreeElementBalance");
    PathChooser.addOption("Drive Straight", "DriveStraight");

    autoTab.add("Location", LocationChooser);
    LocationChooser.setDefaultOption("Left", 0);
    LocationChooser.addOption("Middle", 1);
    LocationChooser.addOption("Right", 2);

    autoTab.add("Elements", ElementsChooser);
    ElementsChooser.addOption("Just Drive", 0);
    ElementsChooser.setDefaultOption("Place 1", 1);
    ElementsChooser.addOption("Place 1, Hold 1", 2);
    ElementsChooser.addOption("Place 2", 3);
    ElementsChooser.addOption("Place 2, Hold 1", 4);

    autoTab.add("Balance?", BalanceChooser);
    BalanceChooser.setDefaultOption("No Balance", 0);
    BalanceChooser.addOption("Balance", 1);

    vertArm.ZeroArm();
    lazySusanSub.zeroPosition();
  }

  private void configureBindings() {
    // april tags auto performance buttons
    new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
        .whileTrue(aprilTagLeft);
    new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON)
        .whileTrue(aprilTagRight);
    new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
        .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON))
        .whileTrue(aprilTagMid);
    // april tags spot choosers begin
    // left top left
    new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
        .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_TOP_LEFT_BUTTON))
        .whileTrue(aprilTagLeftTopLeft);
    // left top right
    new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
        .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_TOP_RIGHT_BUTTON))
        .whileTrue(aprilTagLeftTopRight);
    // left mid left
    new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
        .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_MID_LEFT_BUTTON))
        .whileTrue(aprilTagLeftMidLeft);
    // left mid right
    new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
        .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_MID_RIGHT_BUTTON))
        .whileTrue(aprilTagLeftMidRight);
    // right top left
    new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON)
        .and(new JoystickButton(stickOne, Constants.APRIL_TAG_TOP_LEFT_BUTTON))
        .whileTrue(aprilTagRightTopLeft);
    // right top right
    new JoystickButton(stickOne, Constants.APRIL_TAG_RIGHT_BUTTON)
        .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_TOP_RIGHT_BUTTON))
        .whileTrue(aprilTagRightTopRight);
    //right mid left
    new JoystickButton(stickOne, Constants.APRIL_TAG_RIGHT_BUTTON).and(new JoystickButton(stickTwo, Constants.APRIL_TAG_MID_LEFT_BUTTON))
        .whileTrue(aprilTagRightMidLeft);
    //right mid right
    new JoystickButton(stickOne, Constants.APRIL_TAG_RIGHT_BUTTON).and(new JoystickButton(stickTwo, Constants.APRIL_TAG_MID_RIGHT_BUTTON))
        .whileTrue(aprilTagRightMidRight);
    // april tag spot choosers end

    // This lets a person press single button and open and close the claw every
    // other time.
    new JoystickButton(stickThree, Constants.TOGGLE_CLAW_BUTTON)
        .toggleOnTrue(new TogglePneumatics(pneumatics, !pneumatics.getChannel())); // CHANGETOTHREE

    new JoystickButton(stickFour, Constants.TOGGLE_SUSAN_DIRECTION_BUTTON).toggleOnTrue(new ToggleSusan(lazySusanSub));// CHANGETOTHREE
    new JoystickButton(stickOne, Constants.NO_SLIP_DRIVE_BUTTON).whileTrue(new SlightTurnDrive(mDriveTrain));
    new JoystickButton(stickThree, Constants.ZERO_SUSAN_HEADING_BUTTON)
        .onTrue(new SequentialCommandGroup(
            new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS)
                .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, 0))));
    new JoystickButton(stickThree, Constants.ONE_EIGHTY_SUSAN_HEADING_BUTTON)
        .onTrue(new SequentialCommandGroup(
            new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS)
                .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, Constants.SUSAN_180_ENC_POS))));

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
    new JoystickButton(stickThree, Constants.REACH_OUT_BUTTON).whileTrue(new ControlReach(reachArmSub, () -> 1, 75));
    new JoystickButton(stickThree, Constants.REACH_IN_BUTTON).whileTrue(new ControlReach(reachArmSub, () -> -1, 75));
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
    mPath.setPath(PathChooser.getSelected());

  }

  public Command getAutonomousCommand() {
    // return mAutoContainer.autoRunCommand();
    // return new RunPathAuto(mPath, mDriveTrain);
    // return new WithMarker(mDriveTrain, mPath);
    System.out.println(singlePaths.GetAutoCommand() + "auto");
    Location.setLocation(LocationChooser.getSelected());
    Elements.setElements(ElementsChooser.getSelected());
    BalanceSP.setBalance(BalanceChooser.getSelected());
    return singlePaths.GetAutoCommand();
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
