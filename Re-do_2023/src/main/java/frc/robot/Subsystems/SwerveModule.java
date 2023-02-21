package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private final PIDController anglePIDController;

    private AnalogInput absoluteEncoder;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;


    public SwerveModule(SwerveModuleConstants swerveModuleConstants){
        this.driveMotor = swerveModuleConstants.driveMotor;
        this.angleMotor = swerveModuleConstants.angleMotor;

        this.driveEncoder = swerveModuleConstants.driveEncoder;
        this.angleEncoder = swerveModuleConstants.angleEncoder;

        this.anglePIDController = swerveModuleConstants.anglePIDController;

        this.absoluteEncoder = swerveModuleConstants.absoluteEncoder;
        this.absoluteEncoderReversed = swerveModuleConstants.absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = swerveModuleConstants.absoluteEncoderOffsetRad;

        resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getAnglePosition(){
        return angleEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getAngleVelocity(){
        return angleEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.MAX_SPEED_METERS_PER_SECOND);
        angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public Rotation2d getAngle(){
        return new Rotation2d(getAnglePosition());
    }
}