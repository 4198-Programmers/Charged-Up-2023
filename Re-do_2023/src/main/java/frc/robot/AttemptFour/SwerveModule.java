package frc.robot.AttemptFour;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AttemptFour.Constants.SwerveConstants;

public class SwerveModule {
    //Initializing angle and drive motors
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    
    //Initializing drive Encoder
    private RelativeEncoder driveEncoder;

    //Initializing angle Encoder
    private WPI_CANCoder angleEncoder;

    //Initializing angle PID Controller
    private PIDController anglePID;

    /**
     * Swerve Module 
     * @param driveMotorID CAN ID of the drive motor
     * @param angleMotorID CAN ID of the angle motor
     * @param angleEncoderID CAN ID of the angle encoder(CANCoder)
     * @param angleOffsetDegrees Angle Encoder Offset
     */
    public SwerveModule(int driveMotorID, int angleMotorID, int angleEncoderID, double angleOffsetDegrees){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

        angleEncoder = new WPI_CANCoder(angleEncoderID);

        driveMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        angleEncoder.configFactoryDefault();

        angleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERTED);
        angleMotor.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        angleMotor.setIdleMode(IdleMode.kBrake);

        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        angleEncoder.configMagnetOffset(angleOffsetDegrees);
        angleEncoder.configSensorDirection(SwerveConstants.ANGLE_ENCODER_DIRECTION);
        angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        anglePID = new PIDController(SwerveConstants.ANGLE_KP, SwerveConstants.ANGLE_KI, SwerveConstants.ANGLE_KD);
        anglePID.enableContinuousInput(-180, 180);

        driveMotor.enableVoltageCompensation(Constants.VOLTAGE_COMPENSATION);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
        driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
    }

    /**
     * Return the current state of the module
     * @return module state
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    /**
     * Return the current position of the module
     * @return module position
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    /**
     * Set Module State
     * @param state module state
     */
    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = state;//SwerveModuleState.optimize(state, getState().angle);
        System.out.println("Wanted Angle: "+optimizedState.angle.getDegrees());
        System.out.println("Current Angle: "+ getAngle());
        //double angleOutput = anglePID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());
        double angleOutput = setAngle(optimizedState.angle.getDegrees(), getAngle());
       // System.out.println("Angle Output: " + angleOutput);
        angleMotor.set(angleOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond);
    }

    /**
     * Get Module Angle
     */
    public double getAngle(){
        return angleEncoder.getAbsolutePosition();
    }
    /**
     * Get Module Drive Speed
     */
    public double getDriveSpeed(){
        return getState().speedMetersPerSecond;
    }
    public double getAngleSpeed(){
        return angleEncoder.getVelocity();
    }

    public double setAngle(double wantedAngle, double currentAngle){
        double angleDiff = Math.abs(wantedAngle - currentAngle);
        double speed = (1 - (angleDiff/360)) / 4;
        wantedAngle = wantedAngle == -180 ? 180: wantedAngle;
        currentAngle = currentAngle == -180 ? 180 : currentAngle;
        if(angleDiff < 180){
            if(wantedAngle > 0){
                if(currentAngle > 0){
                    if(wantedAngle > currentAngle){
                        /*  angleDiff < 180
                            wantedAngle > 0
                            currentAngle > 0
                            wantedAngle > currentAngle */
                        speed *= 1;
                    }else{
                        /*  angleDiff < 180
                            wantedAngle > 0
                            currentAngle > 0
                            wantedAngle < currentAngle */
                        speed *= -1;
                    }
                }else{
                    /*  angleDiff < 180
                        wantedAngle > 0
                        currentAngle < 0 */
                    speed *= 1;
                }
            }else{
                if(currentAngle > 0){
                    /*  angleDiff < 180
                        wantedAngle < 0
                        currentAngle > 0 */
                    speed *= -1;
                }
            }
        }else{
            if(wantedAngle > 0){
                if(currentAngle < 0){
                    /*  angleDiff > 180
                        wantedAngle > 0
                        currentAngle < 0 */
                    speed *= -1;
                }
            }else{
                if(currentAngle > 0){
                    /*  angleDiff > 180
                        wantedAngle < 0
                        currentAngle > 0 */
                    speed *= 1;
                }else{
                    if(wantedAngle < currentAngle){
                        /*  wantedAngle < 0
                            currentAngle < 0
                            wantedAngle < currentAngle */
                        speed *= -1;
                    }else{
                        /*  wantedAngle < 0
                            currentAngle < 0
                            currentAngle < wantedAngle */
                        speed *= 1;
                    }
                }
            }
        }
        speed = Math.abs(speed) > Constants.ANGLE_SPEED_DEADBAND ? speed : 0;
        return speed;
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

     private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {

          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}
