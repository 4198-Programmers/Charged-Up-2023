package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax spinMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder spinEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // spinEncoder = spinMotor.getEncoder();

        // spinMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

        // driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);

        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        
    }

}
