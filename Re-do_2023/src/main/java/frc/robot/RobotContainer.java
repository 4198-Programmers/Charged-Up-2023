// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.DriveTrainCom;
import frc.robot.Subsystems.DriveTrain;

public class RobotContainer {
  private final Joystick stickOne = new Joystick(0);
  private final Joystick stickTwo = new Joystick(1);

  private final DriveTrain mDriveTrain = new DriveTrain();

  public RobotContainer() {
    configureBindings();
    mDriveTrain.setDefaultCommand(new DriveTrainCom(
        mDriveTrain,
        () -> -modifyAxis(stickOne.getX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(stickOne.getY()) * -DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(stickTwo.getX()) * -DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

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

  public void resetGyro() {
    mDriveTrain.zeroGyro();
  }
}
