package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private final Rotation2d[] lastAngles;

  private Field2d field;

  public Swerve() {
    gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
   
    zeroGyro();
    //Manually makes empty angles
    lastAngles = new Rotation2d[]{
      new Rotation2d(),
      new Rotation2d(),
      new Rotation2d(),
      new Rotation2d()
    };
   
    
    

   // ADD SWERVE MOD POSITION REMOVE NULL

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }
  public Pose2d resetPose(){
    return new Pose2d();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    // ADD SWERVE MOD POSITION REMOVE NULL
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  private void setModule(int i, SwerveModuleState desiredState){
    mSwerveMods[i].setDesiredState(desiredState, true);
    lastAngles[i] = desiredState.angle;
    
  }
/**Sets the rotation and speed to 0 */
  public void pointWheelsForward(){
    for( int i = 0; i< 4; i++){
      setModule(i, new SwerveModuleState(0, new Rotation2d()));
    }
  }

  public void pointWheelsInward(){
    setModule(0, new SwerveModuleState(0, Rotation2d.fromDegrees(Constants.FRONT_LEFT_ANGLE_INWARD)));
    setModule(1, new SwerveModuleState(0, Rotation2d.fromDegrees(Constants.FRONT_RIGHT_ANGLE_INWARD)));
    setModule(2, new SwerveModuleState(0, Rotation2d.fromDegrees(Constants.BACK_LEFT_ANGLE_INWARD)));
    setModule(3, new SwerveModuleState(0, Rotation2d.fromDegrees(Constants.BACK_RIGHT_ANGLE_INWARD)));
  }

  public void balance(){
    if(gyro.getPitch() > 0.5){
      drive(new Translation2d(0, 1), 0, false, true);
    }
    else if(gyro.getPitch() < -0.5){
      drive(new Translation2d(0, -0.5), 0, false, true);
    }
    else{
      drive(new Translation2d(0, 0), 0, false, false);
    }
  }
  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions()); 
    // Uncomment and fix line above for odometry. Simple needs module positions added. 
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
