package frc.robot;


import frc.robot.commands.CenterTag;
import frc.robot.commands.CheckPhotonTarget;
import frc.robot.commands.DistanceTag;
import frc.robot.commands.FlattenTag;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.commands.AutoMove;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.ControlArm;
import frc.robot.commands.ControlSusan;
import frc.robot.commands.MathDriveCom;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopClaw;
import frc.robot.commands.StopSusan;
import frc.robot.subsystems.GyroSub;
import frc.robot.subsystems.LazySusanSub;
import frc.robot.subsystems.MathDriveTrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VertArm;
import frc.robot.wpiVcontainer.FieldDriveCom;
import frc.robot.wpiVcontainer.FieldDriveSub;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  //photonvision variables
  private final PhotonVision photonVision = new PhotonVision();
  // private final FieldDriveSub fieldDriveTrain = new FieldDriveSub(); //
  // Subsystems
  // private final Pneumatics pneumaticsClaw = new Pneumatics();
  private final MathDriveTrain mathDriveTrain = new MathDriveTrain();
  // private final LazySusanSub lazySusan = new LazySusanSub();
  // private final VertArm vertArm = new VertArm();
  private final GyroSub gyroSub = new GyroSub();

  private final Joystick joystickLeft = new Joystick(0); // Joysticks
  private final Joystick joystickMid = new Joystick(1);
  private final Joystick joystickRight = new Joystick(2);
  private final Trigger clawBTN = new JoystickButton(joystickLeft, 1);
  private final DriveTrain driveTrain = new DriveTrain(); 

  // private final FieldDriveCom fieldDriving = new FieldDriveCom(fieldDriveTrain,
  // joystickLeft, joystickMid);
  private final MathDriveCom mathDriving = new MathDriveCom(mathDriveTrain, joystickLeft, joystickMid);
  // private final StopClaw stopClaw = new StopClaw(pneumaticsClaw);
  // private final OpenClaw openClaw = new OpenClaw(pneumaticsClaw);

  // All the comments are pre-made code that can't be initialized yet

  // private final CloseClaw closeClaw = new CloseClaw(pneumaticsClaw);
  // private final ControlSusan controlSusan = new ControlSusan(lazySusan,
  // joystickRight);
  // private final StopSusan stopSusan = new StopSusan(lazySusan);
  // private final ControlArm controlArm = new ControlArm(vertArm, joystickRight);
  // private final StopArm stopArm = new StopArm(vertArm);

  
  private final Trigger button1 = new JoystickButton(joystickLeft, 1);
  private final Trigger button2 = new JoystickButton(joystickLeft, 2);
  
  public RobotContainer() {
    
  }

  public void initialize() {
    configureButtonBindings();
    // fieldDriveTrain.setDefaultCommand(fieldDriving); //to change driving modes
    // simply switch which is commented out
    mathDriveTrain.setDefaultCommand(mathDriving);
    // vertArm.setDefaultCommand(controlArm);
    // lazySusan.setDefaultCommand(controlSusan);
    // pneumaticsClaw.setDefaultCommand(stopClaw);

  }

  public void updating() {
    gyroSub.displayGyro();
  }

  private void configureButtonBindings() {
    button1.whileTrue(new CheckPhotonTarget(photonVision));   
    button2.whileTrue(new FlattenTag(photonVision, driveTrain).andThen(new CenterTag(photonVision, driveTrain)).andThen(new DistanceTag(photonVision, driveTrain)));
    // clawBTN.toggleOnTrue(closeClaw);
    // clawBTN.toggleOnFalse(openClaw);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
