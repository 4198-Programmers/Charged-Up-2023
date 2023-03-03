package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve;
import java.util.function.Supplier;

public class TeleopSwerve extends CommandBase {
  private final Swerve swerve;
  private final Supplier<Double> xSpeedFunction, ySpeedFunciton, angleSpeedFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  final SlewRateLimiter xLimiter, yLimiter, angleLimiter;



  public TeleopSwerve(Swerve swerve, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> angleSpeedFunction,
  Supplier<Boolean> fieldOrientedFunction){
    this.swerve  = swerve;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunciton = ySpeedFunction;
    this.angleSpeedFunction = angleSpeedFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(4.0);
    this.yLimiter = new SlewRateLimiter(4.0);;
    this.angleLimiter = new SlewRateLimiter(4.0);
    addRequirements(swerve);
  }

  @Override
  public void execute() {
  double xSpeed = xSpeedFunction.get();
  double ySpeed = ySpeedFunciton.get();
  double angleSpeed = angleSpeedFunction.get();
 


  xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.kMaxSpeedMetersPerSecond;
  ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.kMaxSpeedMetersPerSecond;
  angleSpeed = angleLimiter.calculate(angleSpeed) * AutoConstants.kMaxSpeedMetersPerSecond;

  ChassisSpeeds chassisSpeeds;
  if(fieldOrientedFunction.get()){
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angleSpeed, swerve.getRotation2d());
  }else{
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angleSpeed);
  }
  SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
  swerve.setModuleStates(moduleStates);
  

  
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