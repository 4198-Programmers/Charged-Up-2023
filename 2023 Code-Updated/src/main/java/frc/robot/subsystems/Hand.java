package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase{
    CANSparkMax handMotor = new CANSparkMax(Constants.HAND_MOTOR, MotorType.kBrushless);

    public void setSpeed(double speed){
        handMotor.set(speed);
    }
}
