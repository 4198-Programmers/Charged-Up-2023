package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTesting extends SubsystemBase{
    CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);

    public void go(double speed){
        motor.set(speed);
    }
    
}
