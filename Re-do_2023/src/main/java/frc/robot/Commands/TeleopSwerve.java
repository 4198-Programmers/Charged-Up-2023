package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;
import java.util.function.Supplier;

public class TeleopSwerve extends CommandBase {
  private final Swerve swerve;
  private final Supplier<Double> xSpeedFunction, ySpeedFunciton, angleSpeedFunction;
  private final Supplier<Boolean> fieldOrientedFunction;


  public TeleopSwerve(Swerve swerve, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> angleSpeedFunction,
  Supplier<Boolean> fieldOrientedFunction){
    this.swerve  = swerve;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunciton = ySpeedFunction;
    this.angleSpeedFunction = angleSpeedFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
  double xSpeed = xSpeedFunction.get();
  double ySpeed = ySpeedFunciton.get();
  double angleSpeed = angleSpeedFunction.get();

  swerve.drive(xSpeed, ySpeed, angleSpeed, fieldOrientedFunction.get());


  }
  @Override
  public void end(boolean interrupted) {
      swerve.stopModules();
  }
  @Override
  public boolean isFinished() {
      return false;
  }
}