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
  private final Joystick stickOne = new Joystick(Constants.PORT_ZERO);
  private final Joystick stickTwo = new Joystick(Constants.PORT_ONE);
  private final Joystick stickThree = new Joystick(Constants.PORT_TWO);

  public RobotContainer() {
    configureBindings();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    pipeline.setDouble(0);

  }


  public void initShuffleboard() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Choices");
  }

  private void configureBindings() {
  }

  public void initializeAuto() {

  }

  private static double deadband(double value, double deadband) {
    if(Math.abs(value) > deadband) {
      if(value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    //deadband the value
    value = deadband(value, 0.1);
    //square the axis?
    value = Math.copySign(value * value, value);
    
    return value;
  }
}