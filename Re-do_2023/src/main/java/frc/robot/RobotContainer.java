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
import frc.robot.AttemptOne.SwerveDrive;
import frc.robot.AttemptOne.SwerveSubsystem;
import frc.robot.AttemptTwo.Swerve;
import frc.robot.AttemptTwo.TeleopSwerve;

public class RobotContainer {
//Joystics
  private final Joystick stickOne = new Joystick(Constants.PORT_ZERO);
  private final Joystick stickTwo = new Joystick(Constants.PORT_ONE);
  private final Joystick stickThree = new Joystick(Constants.PORT_TWO);
//Subsystems
  private final Swerve swerveSubsystem = new Swerve();
  public RobotContainer() {
    configureBindings();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    pipeline.setDouble(0);

    /*
     * There is a deadband in the drive function itself, 
     * so it does not need a deadband here.
     */
    boolean fieldRelative = true;
    boolean openLoop = true;
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(
      swerveSubsystem, () -> stickOne.getX(), 
      () -> stickOne.getY(), 
      () -> stickTwo.getX(), fieldRelative, openLoop));

  }


  public void initShuffleboard() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Choices");
  }

  private void configureBindings() {
  }

  public void initializeAuto() {

  }
}