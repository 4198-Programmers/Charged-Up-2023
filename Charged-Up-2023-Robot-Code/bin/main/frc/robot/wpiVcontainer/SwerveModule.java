package frc.robot.wpiVcontainer;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private CANSparkMax driveMotor; // motor that drives the wheel
    private CANSparkMax spinMotor;// motor that spins the wheel around to face different ways
    private CANCoder spinEncoder; // cancoder, just named enco
    private RelativeEncoder driveEncoder; // encoder for drvie motor
    private double spinOffset; // distance from our zero to the magnetic 0 in the cancoders
    private double spinPos; // output of the cancoder, from 0-360
    private Rotation2d rotation; // turns cancoder value to valid rotation value for later use

    public SwerveModule(int driveID, int spinID, int CANCoderID, int driveEncoderID, double CANCoderZeroOffset) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
        spinEncoder = new CANCoder(CANCoderID);
        driveEncoder = driveMotor.getEncoder();
        spinOffset = CANCoderZeroOffset;

        if (spinPos < 0) {
            spinPos = 360 + spinEncoder.getAbsolutePosition() - spinOffset; // makes sure that the number stays between
                                                                            // 0-360 instead of going negative
        } else {
            spinPos = spinEncoder.getAbsolutePosition() - spinOffset;
        }
        rotation = Rotation2d.fromDegrees(spinPos);

    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // allows for optimization
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, rotation);
    }

}
