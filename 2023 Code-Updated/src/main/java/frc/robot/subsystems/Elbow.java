package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase{
    CANSparkMax elbowMotor = new CANSparkMax(Constants.ELBOW_MOTOR, MotorType.kBrushless);

    public void setSpeed(double speed){
        elbowMotor.set(speed);
    }
}
