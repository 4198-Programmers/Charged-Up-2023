package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.SwerveModuleConstants;

public class SwerveModule {
public int moduleNumber;
private Rotation2d lastAngle;
private Rotation2d angleOffset;

private CANSparkMax angleMotor;
private CANSparkMax driveMotor;

private RelativeEncoder driveEncoder;
private RelativeEncoder integratedAngleEncoder;
private CANCoder angleEncoder;

private final SparkMaxPIDController driveController;
private final SparkMaxPIDController angleController;

private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
    this.moduleNumber = moduleNumber;
    angleOffset= moduleConstants.angleOffset;

    angleEncoder = new CANCoder(moduleConstants.cancoderID);

    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();

    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();

    lastAngle = getState().angle;
}

public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
}

public  void resetToAbsolute(){
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
}

private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_SPEED_METERS_PER_SECOND;
        driveMotor.set(percentOutput);
    }else{
        driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0 , feedforward.calculate(desiredState.speedMetersPerSecond));
    }
}

private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_SPEED_METERS_PER_SECOND * 0.01))? lastAngle : desiredState.angle;
    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
}

private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
}
public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
}
public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
}

}
