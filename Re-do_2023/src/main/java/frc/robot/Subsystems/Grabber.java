package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase{
    CANSparkMax grabberMotor = new CANSparkMax(Constants.GRABBER_MOTOR_ID, MotorType.kBrushed);

    public void setSpeed(double speed){
        grabberMotor.set(speed);
    }
}
