package frc.robot.Subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

public class Swerve extends SubsystemBase {
  
private final SwerveModule frontLeft = new SwerveModule(
  Mod0.driveMotorID, 
  Mod0.angleMotorID, 
  Mod0.canCoderID, 
  Mod0.angleOffset);

  private final SwerveModule frontRight = new SwerveModule(
  Mod1.driveMotorID, 
  Mod1.angleMotorID, 
  Mod1.canCoderID, 
  Mod1.angleOffset);
  private final SwerveModule backLeft = new SwerveModule(
  Mod2.driveMotorID, 
  Mod2.angleMotorID, 
  Mod2.canCoderID, 
  Mod2.angleOffset);

  private final SwerveModule backRight = new SwerveModule(
  Mod3.driveMotorID, 
  Mod3.angleMotorID, 
  Mod3.canCoderID, 
  Mod3.angleOffset);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private final SwerveModulePosition[] positions = new SwerveModulePosition[]{
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  };
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRotation2d(), positions);

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();     
   }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), positions, pose);
  }

  public void printModuleAngles(){
    System.out.println("Front Left Module: " + frontLeft.getAbsoluteEncoderPosition());
    System.out.println("Front Right Module: " + frontRight.getAbsoluteEncoderPosition());
    System.out.println("Back Left Module: " + backLeft.getAbsoluteEncoderPosition());
    System.out.println("Back Right Module: " + backRight.getAbsoluteEncoderPosition());
  }

  public double balance(){
    double speed;
    if(gyro.getPitch() > 0.5){
      speed = Constants.BALANCE_SPEED;
    }
    else if(gyro.getPitch() < -0.5){
      speed = -Constants.BALANCE_SPEED;
    }
    else{
      speed = 0;
    }
    return speed;
  }

  @Override
  public void periodic() {
      odometer.update(getRotation2d(), positions);
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
      SmartDashboard.putNumber("Front Left Angle", frontLeft.getAbsoluteEncoderPosition());
      SmartDashboard.putNumber("Front Right Angle", frontRight.getAbsoluteEncoderPosition());
      SmartDashboard.putNumber("Back Left Angle", backLeft.getAbsoluteEncoderPosition());
      SmartDashboard.putNumber("Back Right Anglr", backRight.getAbsoluteEncoderPosition());
      
  }




  //Using ModuleTest
  // public static final CTREConfigs ctreConfigs = new CTREConfigs();
  // public SwerveDriveKinematics kinematics = Constants.Swerve.swerveKinematics;

  // private final SwerveModuleTest[] modules;
  // private final Rotation2d[] lastAngles;

  // private final SwerveDrivePoseEstimator odometer;
  // private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  
  // private final Field2d field = new Field2d();

  // public Swerve(){
  //   SmartDashboard.putData("Field", field);
  //   SmartDashboard.putData("resetOdometry", new InstantCommand(() -> this.resetOdometry(getPose())));

  //   modules = new SwerveModuleTest[4];
  //   lastAngles = new Rotation2d[]{
  //     new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};

  //     modules[0] = new SwerveModuleTest(
  //       Mod0.moduleName,
  //       Mod0.driveMotorID,
  //       Mod0.angleMotorID,
  //       Mod0.canCoderID,
  //       Mod0.rotationOffSet);
  //     modules[1] = new SwerveModuleTest(
  //       Mod1.moduleName,
  //       Mod1.driveMotorID,
  //       Mod1.angleMotorID,
  //       Mod1.canCoderID,
  //       Mod1.rotationOffSet);
  //     modules[2] = new SwerveModuleTest(
  //       Mod2.moduleName,
  //       Mod2.driveMotorID,
  //       Mod2.angleMotorID,
  //       Mod2.canCoderID,
  //       Mod2.rotationOffSet);
  //     modules[3] = new SwerveModuleTest(
  //       Mod3.moduleName,
  //       Mod3.driveMotorID,
  //       Mod3.angleMotorID,
  //       Mod3.canCoderID,
  //       Mod3.rotationOffSet);

  //       odometer = new SwerveDrivePoseEstimator(kinematics, 
  //       getGryoRotation(), 
  //       getModulePositions(), 
  //       getPose()
  //       );
  // }

  // public void zeroGyro(){
  //   gyro.zeroYaw();
  //   odometer.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  // }

  // public void zeroGyro(double angleOffset){
  //   gyro.setAngleAdjustment(angleOffset);
  //   new Rotation2d();
  //   new Rotation2d();
  //   odometer.resetPosition(Rotation2d.fromDegrees(angleOffset), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleOffset)));
  // }

  // public Rotation2d getGryoRotation(){
  //   return gyro.getRotation2d();
  // }
  // public Rotation2d getGryoPitch(){
  //   return Rotation2d.fromDegrees(gyro.getPitch());
  // }

  // public SwerveModulePosition[] getModulePositions(){
  //   SwerveModulePosition[] positions = new SwerveModulePosition[4];
  //   for(int i = 0; i<4; i++) positions[i] = modules[i].getPosition();
  //   return positions;
  // }

  // public SwerveModuleState[] getModuleStates(){
  //   SwerveModuleState[] states = new SwerveModuleState[4];
  //   for(int i = 0; i < 4; i++) states[i] = modules[i].getState();
  //   return states;
  // }

  // public Pose2d getPose(){
  //   return odometer.getEstimatedPosition();
  // }

  // public void resetOdometry(Pose2d pose){
  //   odometer.resetPosition(pose.getRotation(), getModulePositions(), pose);
  // }

  // public void resetOdometryFromVision(Pose2d pose){
  //   odometer.resetPosition(getGryoRotation(), getModulePositions(), pose);
  // }

  // public void displayFieldTrajectory(PathPlannerTrajectory trajectory){
  //   field.getObject("trajectory").setTrajectory(trajectory);
  // }
  // public void pointWheelsForward(){
  //   for(int i =0; i < 4; i++){
  //     setModule(i, new SwerveModuleState(0, new Rotation2d()));
  //   }
  // }

  // public void pointWheelsInward(){
  //   setModule(0, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   setModule(1, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   setModule(2, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   setModule(3, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));

  // }

  // public void stop(){
  //   for (int i = 0; i < 4; i++){
  //     modules[i].stop();
  //   }
  // }

  // public void setModuleStates(SwerveModuleState[] desiredStates){
  //   SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.kMaxSpeedMetersPerSecond);
  //   for(int i = 0; i < 4; i ++){
  //     setModule(i, desiredStates[i]);
  //   }
  // }

  // private void setModule(int i, SwerveModuleState desiredState){
  //   modules[i].setDesiredState(desiredState);
  //   lastAngles[i] = desiredState.angle;
  // }

  // public void updateOdometry(){
  //   odometer.updateWithTime(Timer.getFPGATimestamp(), getGryoRotation(), getModulePositions());
  // }

  // @Override
  // public void periodic() {
  //     updateOdometry();
  //     field.setRobotPose(odometer.getEstimatedPosition());
  //     SmartDashboard.putNumber("Robot Angle", getGryoRotation().getDegrees());
  //     SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  // }

}
