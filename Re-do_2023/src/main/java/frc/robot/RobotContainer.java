// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AutoReach;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ControlArm;
import frc.robot.Commands.ControlReach;
import frc.robot.Commands.ControlSusan;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.AutoDriveBalance;
import frc.robot.Commands.AutoDriveDock;
import frc.robot.Commands.RunIntake;
import frc.robot.Commands.SlightTurnDrive;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.PathHolder;
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
import frc.robot.Subsystems.SinglePaths.SideChoice;
import frc.robot.Tags.CenterSusanPhoton;
import frc.robot.Tags.PhotonVision;
import frc.robot.Tags.TagFollower;

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
  private final PathHolder mPath = new PathHolder(vertArm, pneumatics, reachArmSub, lazySusanSub);
  private final Intake intakeSub = new Intake();
  private final PhotonVision visionSub = new PhotonVision();
  private final SinglePaths singlePaths = new SinglePaths(mDriveTrain, vertArm, lazySusanSub, pneumatics, intakeSub,
      reachArmSub, visionSub);
  private LEDs leds = new LEDs();
  UsbCamera cam = CameraServer.startAutomaticCapture();

  // private AutoContainer mAutoContainer = new AutoContainer(mDriveTrain,
  // lazySusanSub, pneumatics, reachArmSub, vertArm);
  // private final SendableChooser<Location> LocationChooser = new
  // SendableChooser<>();
  // private final SendableChooser<AutoType> AutoChooser = new
  // SendableChooser<>();
  // private final SendableChooser<LevelPriority> LevelChooser = new
  // SendableChooser<>();
  // private final SendableChooser<String> PathChooser = new SendableChooser<>();
  public final SendableChooser<Integer> LocationChooser = new SendableChooser<>();
  public final SendableChooser<Integer> ElementsChooser = new SendableChooser<>();
  public final SendableChooser<Integer> BalanceChooser = new SendableChooser<>();
  public final SendableChooser<Integer> SideChooser = new SendableChooser<>();
  public final SendableChooser<Integer> AutoChooser = new SendableChooser<>();


  // private final SequentialCommandGroup aprilTagLeft = new
  // SusanHead(lazySusanSub, 0)
  // .andThen(new TagFollower(photonVision, mDriveTrain,
  // Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT,
  // Constants.WANTED_DISTANCE_LEFT));

  // private final SequentialCommandGroup aprilTagRight = new
  // SusanHead(lazySusanSub, 0)
  // .andThen(new TagFollower(photonVision, mDriveTrain,
  // Constants.WANTED_YAW_RIGHT, Constants.WANTED_SKEW_RIGHT,
  // Constants.WANTED_DISTANCE_RIGHT));

  // private final SequentialCommandGroup aprilTagMid = new
  // SusanHead(lazySusanSub, 0)
  // .andThen(new TagFollower(photonVision, mDriveTrain,
  // Constants.WANTED_YAW_MID, Constants.WANTED_SKEW_MID,
  // Constants.WANTED_DISTANCE_MID));

  public RobotContainer() {
    configureBindings();
    reachArmSub.zeroEncoder();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    pipeline.setDouble(0);
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickOne.getY()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.7,
        true)); // ATTENTION These values were multiplied by Oren to make the bot not die while
                // testing the three * .5 terms should be deleted

    // reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () ->
    // -stickThree.getRawAxis(1), 75)); //CHANGETOTHREE
    reachArmSub.setDefaultCommand(new ControlReach(reachArmSub, () -> 0, 0));
    pneumatics.Pressurize();
    new zeroHeading(mDriveTrain); // This sets the robot front to be the forward direction
    // pneumatics.setDefaultCommand(new TogglePneumatics(pneumatics, false));
    vertArm.setDefaultCommand(new ControlArm(vertArm, () -> modifyVertArm(stickThree.getRawAxis(1)), 100));
    lazySusanSub.setDefaultCommand(
        new ControlSusan(lazySusanSub, () -> smallerModifyAxis(-stickThree.getX()), 50));// CHANGETOTHREE
    lazySusanSub.mode(IdleMode.kBrake);
    intakeSub.setDefaultCommand(new RunIntake(intakeSub, 0));
  }

  private final SequentialCommandGroup elementTopLeft = new SequentialCommandGroup(
      new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, 0)
          .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.PLACE_TOP_VERT))
          .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, Constants.LEFT_TOP_PLACEMENT_SUSAN))
          .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, Constants.TOP_REACH_PLACEMENT)));

  private final SequentialCommandGroup elementMidLeft = new SequentialCommandGroup(
      new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, 0)
          .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.PLACE_MID_VERT))
          .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, Constants.LEFT_MID_PLACEMENT_SUSAN))
          .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, Constants.MID_REACH_PLACEMENT)));

  private final SequentialCommandGroup elementTopRight = new SequentialCommandGroup(
      new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, 0)
          .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.PLACE_TOP_VERT))
          .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, Constants.RIGHT_TOP_PLACEMENT_SUSAN))
          .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, Constants.TOP_REACH_PLACEMENT)));

  private final SequentialCommandGroup elementMidRight = new SequentialCommandGroup(
      new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, 0)
          .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.PLACE_MID_VERT))
          .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, Constants.RIGHT_MID_PLACEMENT_SUSAN))
          .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, Constants.MID_REACH_PLACEMENT)));

  private final SequentialCommandGroup upToSubStation = new SequentialCommandGroup(
      new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.SUBSTATION_UP_POS_VERT));

  private final SequentialCommandGroup autoPlaceThenBalance = new AutoVert(vertArm, 0.25, 6)
  .andThen(new AutoDrive(mDriveTrain, 0, 0, 0, 1000))
  .andThen(elementTopRight)
  .andThen(new RunIntake(intakeSub, Constants.INTAKE_OUT_SPEED))
  .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, 0))
  .andThen(new ZeroSusan(lazySusanSub))
  .andThen(new AutoDriveDock(mDriveTrain, -0.25, 0, 0))
  .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 1))
  .andThen(new AutoDriveBalance(mDriveTrain, -0.25, 0, 0));

  public void initShuffleboard() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Choices");
    // autoTab.add("Autonomous", PathChooser);
    // PathChooser.setDefaultOption("Left One Element No Balance",
    // "LeftOneElement");
    // PathChooser.addOption("Left Two Element No Balance", "LeftTwoElement");
    // PathChooser.addOption("Left One Element Balance", "LeftOneElementBalance");
    // PathChooser.addOption("Left Two Element Balance", "LeftTwoElementBalance");
    // PathChooser.addOption("Left Three Element Balance",
    // "LeftThreeElementBalance");
    // PathChooser.addOption("Mid One Element No Balance", "MiddleOneElement");
    // PathChooser.addOption("Mid Two Element No Balance", "MiddleTwoElement");
    // PathChooser.addOption("Mid One Element Balance", "MiddleOneElementBalance");
    // PathChooser.addOption("Mid Two Element Balance", "MiddleTwoElementBalance");
    // PathChooser.addOption("Mid Three Element Balance",
    // "MiddleThreeElementBalance");
    // PathChooser.addOption("Right One Element No Balance", "RightOneElement");
    // PathChooser.addOption("Right Two Element No Balance", "RightTwoElement");
    // PathChooser.addOption("Right One Element No Balance",
    // "RightOneElementBalance");
    // PathChooser.addOption("Right Two Element No Balance",
    // "RightTwoElementBalance");
    // PathChooser.addOption("Right Three Element No Balance",
    // "RightThreeElementBalance");
    // PathChooser.addOption("Drive Straight", "DriveStraight");

    // autoTab.add("Location", LocationChooser);
    // LocationChooser.setDefaultOption("Left", 0);
    // LocationChooser.addOption("Right", 1);
    // LocationChooser.addOption("Middle", 2);

    // autoTab.add("Elements", ElementsChooser);
    // ElementsChooser.addOption("Just Drive", 0);
    // ElementsChooser.setDefaultOption("Place 1", 1);
    // ElementsChooser.addOption("Place 1, Hold 1", 2);
    // ElementsChooser.addOption("Place 2", 3);
    // ElementsChooser.addOption("Place 2, Hold 1", 4);
    // ElementsChooser.addOption("Just Place", 5);
    // ElementsChooser.addOption("Charge Staion", 6);
    // ElementsChooser.addOption("Place Drive Charge", 7);
    // ElementsChooser.addOption("Place Drive Charge mid", 8);


    // autoTab.add("Balance?", BalanceChooser);
    // BalanceChooser.setDefaultOption("No Balance", 0);
    // BalanceChooser.addOption("Balance", 1);

    // autoTab.add("Color", SideChooser);
    // SideChooser.setDefaultOption("Red", 0);
    // SideChooser.addOption("Blue", 1);

    autoTab.add("Auto", AutoChooser);
    AutoChooser.setDefaultOption("Middle Auto", 0);
    AutoChooser.addOption("Blue Left Auto, Charge", 1);
    AutoChooser.addOption("Just Place, No Drive", 2);
    AutoChooser.addOption("Red Right Auto, Charge", 3);
    AutoChooser.addOption("Blue Right Auto, Charge", 4);
    AutoChooser.addOption("Red Left Auto, Charge", 5);
    AutoChooser.addOption("Place + Drive, No Charge", 6);
    AutoChooser.addOption("Just Drive, No Place, No Charge, R/L Only", 7);

    vertArm.ZeroArm();
    lazySusanSub.zeroPosition();
  }

  private void configureBindings() {
    // april tags auto performance buttons
    // new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
    // .whileTrue(aprilTagLeft);
    // new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON)
    // .whileTrue(aprilTagRight);
    // new JoystickButton(stickOne, Constants.APRIL_TAG_LEFT_BUTTON)
    // .and(new JoystickButton(stickTwo, Constants.APRIL_TAG_RIGHT_BUTTON))
    // .whileTrue(aprilTagMid);

    // This lets a person press single button and open and close the claw every
    // other time.
    new JoystickButton(stickThree, Constants.TOGGLE_CLAW_BUTTON)
        .toggleOnTrue(new TogglePneumatics(pneumatics, !pneumatics.getChannel())); // CHANGETOTHREE

    new JoystickButton(stickThree, Constants.SLOW_SUSAN_BUTTON)
        .whileTrue(new ControlSusan(lazySusanSub, () -> smallerModifyAxis(-stickThree.getX()), 15));
    new JoystickButton(stickThree, Constants.INTAKE_BUTTON)
        .whileTrue(new RunIntake(intakeSub, 0.7));
    new JoystickButton(stickThree, Constants.OUTTAKE_BUTTON)
        .whileTrue(new RunIntake(intakeSub, -0.7));

    new JoystickButton(stickTwo, Constants.TEST_SUSAN_PHOTON)
        .whileTrue(new CenterSusanPhoton(visionSub, mDriveTrain, lazySusanSub, 0, 0, 1.2));
    new JoystickButton(stickTwo, Constants.TEST_DRIVE_CENTER_PHOTON)
        .whileTrue(new TagFollower(visionSub, mDriveTrain, 0, 0, 1.2));

    new JoystickButton(stickTwo, Constants.NO_SLIP_DRIVE_BUTTON).whileTrue(new SlightTurnDrive(mDriveTrain));
    new JoystickButton(stickThree, Constants.ZERO_SUSAN_BUTTON)
        .onTrue(new SequentialCommandGroup(
            new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS)
                .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED, 0))));

    // This resets the robot to field orientation and sets the current front of the
    // robot to the forward direction
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).onTrue(new zeroHeading(mDriveTrain));
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).onTrue(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickOne.getY()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .7,
        true));

    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnFalse(new zeroHeading(mDriveTrain));
    new JoystickButton(stickOne, Constants.FIELD_ORIENTATION_BUTTON).toggleOnFalse(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickOne.getY()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .7,
        true));

    // This makes the front of the robot always the forward direction.
    // When fieldOrientation is false, it is in robotOrientation.
    new JoystickButton(stickOne, Constants.ROBOT_ORIENTATION_BUTTON).onTrue(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .7,
        false));
    new JoystickButton(stickOne, Constants.ROBOT_ORIENTATION_BUTTON).toggleOnFalse(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 1,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .7,
        false));

    // Make sure susan is set to a low value because it spins really fast. It has to
    // be at least under 0.3, most likely.
    new JoystickButton(stickThree, Constants.REACH_OUT_BUTTON).whileTrue(new ControlReach(reachArmSub, () -> 1, 75));
    new JoystickButton(stickThree, Constants.REACH_IN_BUTTON).whileTrue(new ControlReach(reachArmSub, () -> -1, 75));
    new JoystickButton(stickThree, Constants.PLACE_TOP_LEFT_BTN).whileTrue(elementTopLeft);
    new JoystickButton(stickThree, Constants.PLACE_MID_LEFT_BTN).whileTrue(elementMidLeft);
    new JoystickButton(stickThree, Constants.PLACE_TOP_RIGHT_BTN).whileTrue(elementTopRight);
    new JoystickButton(stickThree, Constants.PLACE_MID_RIGHT_BIN).whileTrue(elementMidRight);
    new JoystickButton(stickThree, Constants.SUSTATION_UP_BUTTON).whileTrue(upToSubStation);
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
    // mPath.setPath(PathChooser.getSelected());

  }

  public Command getAutonomousCommand() {
    // return mAutoContainer.autoRunCommand();
    // return new RunPathAuto(mPath, mDriveTrain);
    // return new WithMarker(mDriveTrain, mPath);
    // Location.setLocation(LocationChooser.getSelected());
    // Elements.setElements(ElementsChooser.getSelected());
    // BalanceSP.setBalance(BalanceChooser.getSelected());
    // SideChoice.setSide(SideChooser.getSelected());

    // return autoPlaceThenBalance;
    
    singlePaths.setAutoChoice(AutoChooser.getSelected());
    System.out.println(singlePaths.GetAutoCommand() + "auto");
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

  private static double smallerModifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);
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
