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
import frc.robot.AttemptFour.SwerveDrive;
import frc.robot.AttemptFour.SwerveSubsystem;

public class RobotContainer {
//Joystics
  private final Joystick stickOne = new Joystick(Constants.PORT_ZERO);
  private final Joystick stickTwo = new Joystick(Constants.PORT_ONE);
  private final Joystick stickThree = new Joystick(Constants.PORT_TWO);
//Subsystems
  //private final Swerve swerveSubsystem = new Swerve();
  //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public RobotContainer() {
    configureBindings();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    pipeline.setDouble(0);

    /*
     * There is a deadband in the drive function itself, 
     * so it does not need a deadband here.
     */

     swerveSubsystem.setDefaultCommand(new SwerveDrive(
      swerveSubsystem, 
      () -> stickOne.getRawAxis(Constants.X_AXIS), 
      () -> stickOne.getRawAxis(Constants.Y_AXIS), 
      () -> stickTwo.getRawAxis(Constants.X_AXIS), 
      () -> true));
    // boolean fieldRelative = true;
    // boolean openLoop = true;
    // swerveSubsystem.setDefaultCommand(new TeleopSwerve(
    //   swerveSubsystem, () -> stickOne.getRawAxis(Constants.X_AXIS), 
    //   () -> stickOne.getRawAxis(Constants.Y_AXIS), 
    //   () -> stickTwo.getRawAxis(Constants.X_AXIS), fieldRelative, openLoop));

    // swerveSubsystem.setDefaultCommand(new SwerveDrive(swerveSubsystem, 
    // () -> stickOne.getRawAxis(Constants.X_AXIS), 
    // () -> stickOne.getRawAxis(Constants.Y_AXIS), 
    // () -> stickTwo.getRawAxis(Constants.X_AXIS), true));
  }


  public void initShuffleboard() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Choices");
  }

  private void configureBindings() {
  }

  public void initializeAuto() {

  }
}