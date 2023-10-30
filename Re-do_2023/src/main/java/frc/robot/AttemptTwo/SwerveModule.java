package frc.robot.AttemptTwo;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.AttemptTwo.lib.CTREModuleState;
import frc.robot.AttemptTwo.lib.Conversions;
import frc.robot.AttemptTwo.lib.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    private PIDController angleController;
    private SparkMaxPIDController angleMaxPIDController;
    private RelativeEncoder angleRelativeEncoder;
    private RelativeEncoder driveRelativeEncoder;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleMaxPIDController = angleMotor.getPIDController();

        angleRelativeEncoder = angleMotor.getEncoder();
        driveRelativeEncoder = driveMotor.getEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if(isOpenLoop){
            driveMotor.set(desiredState.speedMetersPerSecond / Constants.DRIVE_MAX_SPEED);
        }else{
            driveMotor.set(Conversions.MPSToNeo(desiredState.speedMetersPerSecond, Constants.DRIVE_WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO));
        }
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DRIVE_MAX_SPEED * 0.02)) ? lastAngle : desiredState.angle.getDegrees();
        angleMotor.set(angleController.calculate(lastAngle, Conversions.degreesToNeo(angle)));
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToNeo(getCancoder().getDegrees() - angleOffset);
        angleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.canCoderConfig);
    }

    private void configAngleMotor(){
        angleMaxPIDController.setP(Constants.ANGLE_KP);
        angleMaxPIDController.setI(Constants.ANGLE_KI);
        angleMaxPIDController.setD(Constants.ANGLE_KD);
        angleMaxPIDController.setFF(Constants.ANGLE_FF);
        angleMotor.setInverted(Constants.ANGLE_MOTOR_INVERTED);
        resetToAbsolute();
    }
    private void configDriveMotor(){
        driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERTED);
    }

    public Rotation2d getCancoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.neoToMPS(driveMotor.get(), Constants.DRIVE_WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.neoToDegrees(angleRelativeEncoder.getPosition()));
        return new SwerveModuleState(velocity, angle);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.neoToDegrees(angleRelativeEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.neoToMeters(driveRelativeEncoder.getPosition(), Constants.DRIVE_WHEEL_CIRCUMFERENCE), getAngle());
    }
}
