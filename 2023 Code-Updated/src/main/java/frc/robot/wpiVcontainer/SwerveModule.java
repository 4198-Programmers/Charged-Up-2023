package frc.robot.wpiVcontainer;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax spinMotor;
    private CANCoder spinEncoder;
    private RelativeEncoder driveEncoder;
    private double spinOffset;
    private double spinPos;
    private Rotation2d rotation;

    public SwerveModule(int driveID, int spinID, int CANCoderID, int driveEncoderID, double CANCoderZeroOffset) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
        spinEncoder = new CANCoder(CANCoderID);
        driveEncoder = driveMotor.getEncoder();
        spinOffset = CANCoderZeroOffset;

        spinPos = spinEncoder.getAbsolutePosition()-spinOffset;
        rotation = Rotation2d.fromDegrees(spinPos);

    }

    public void setDesiredState(SwerveModuleState desiredState){
        //allows for optimization
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, rotation);
    }

}
