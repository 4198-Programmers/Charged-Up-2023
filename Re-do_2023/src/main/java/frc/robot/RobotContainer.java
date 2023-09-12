// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotContainer {
  private final Joystick stickOne = new Joystick(0);
  private final Joystick stickTwo = new Joystick(1);
  private final Joystick stickThree = new Joystick(2);
  // private final Joystick stickFour = new Joystick(3);

  // private AutoContainer mAutoContainer = new AutoContainer(mDriveTrain,
  // lazySusanSub, pneumatics, reachArmSub, vertArm);
  // private final SendableChooser<Location> LocationChooser = new
  // SendableChooser<>();
  // private final SendableChooser<AutoType> AutoChooser = new
  // SendableChooser<>();
  // private final SendableChooser<LevelPriority> LevelChooser = new
  // SendableChooser<>();
  // private final SendableChooser<String> PathChooser = new SendableChooser<>();

  // private final SequentialCommandGroup aprilTagLeft =
  // new TagFollower(photonVision, mDriveTrain,
  // Constants.WANTED_YAW_LEFT, Constants.WANTED_SKEW_LEFT,
  // Constants.WANTED_DISTANCE_LEFT));

  // private final SequentialCommandGroup aprilTagRight =
  // new TagFollower(photonVision, mDriveTrain,
  // Constants.WANTED_YAW_RIGHT, Constants.WANTED_SKEW_RIGHT,
  // Constants.WANTED_DISTANCE_RIGHT));

  public RobotContainer() {
    configureBindings();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    pipeline.setDouble(0);

  }

 
  // private final SequentialCommandGroup elementTopRightAuto = new
  // SequentialCommandGroup(
  // new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.PLACE_TOP_VERT)
  // .andThen(new AutoSusan(lazySusanSub, Constants.AUTO_SUSAN_SPEED,
  // Constants.RIGHT_TOP_PLACEMENT_SUSAN))
  // .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED,
  // Constants.TOP_REACH_RIGHT_PLACEMENT)));

  // private final SequentialCommandGroup autoPlaceTopLeftThenBlance = new
  // SequentialCommandGroup(
  // new AutoVert(vertArm, 0.25, 6)
  // .andThen(new AutoDrive(mDriveTrain, -0.25, 0, 0, 3000))
  // .andThen(elementTopRightAuto)
  // .andThen(new AutoRunIntake(intakeSub, Constants.INTAKE_OUT_SPEED))
  // .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, 0))
  // .andThen(new AutoDriveDock(mDriveTrain, 0.6, 0, 0))
  // .andThen(new TimedAuto(mDriveTrain, 5000, 0.25, 0, 0))
  // .andThen(new TimedAuto(mDriveTrain, 1000, -0.25, 0, 0))
  // .andThen(new SlightTurnDrive(mDriveTrain)));


  // private final SequentialCommandGroup autoPlaceThenBalance = new
  // AutoVert(vertArm, 0.25, 6)
  // .andThen(new AutoDrive(mDriveTrain, 0, 0, 0, 200))
  // .andThen(elementTopRight)
  // .andThen(new RunIntake(intakeSub, Constants.INTAKE_OUT_SPEED))
  // .andThen(new AutoReach(reachArmSub, Constants.AUTO_REACH_SPEED, 0))
  // .andThen(new ZeroSusan(lazySusanSub))
  // .andThen(new AutoDriveDock(mDriveTrain, -0.25, 0, 0))
  // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 1))
  // .andThen(new AutoDriveBalance(mDriveTrain, -0.25, 0, 0));

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

  
  }

  private void configureBindings() {
  }

  public void initializeAuto() {
    // mPath.setPath(PathChooser.getSelected());

  }



}
