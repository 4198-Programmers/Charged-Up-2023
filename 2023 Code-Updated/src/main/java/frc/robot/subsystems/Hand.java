package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hand extends SubsystemBase{
    CANSparkMax handMotor = new CANSparkMax(Constants.HAND_MOTOR, MotorType.kBrushless);
    RelativeEncoder handEncoder = handMotor.getEncoder();

    public void setSpeed(double speed){
        handMotor.set(speed);
    }
    public double getLocation(){
        return handEncoder.getPosition();
    }
    public void setSpeedWithLimits(double speed){
        double expectedSpeed = speed;
        if(speed > 0 && getLocation() >= Constants.HAND_UPPER_LIMIT){
            expectedSpeed = 0;
        }
        else if(speed < 0 && getLocation() <= Constants.HAND_LOWER_LIMIT){
            expectedSpeed = 0;
        }
        handMotor.set(expectedSpeed);
    }
}
