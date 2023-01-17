package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase{
    CANSparkMax shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushless);
    RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();
    public void setSpeed(double speed){
        shoulderMotor.set(speed);
    }
    public double location(){
        return shoulderEncoder.getPosition();
    }
    public void boundaries(double speed){
        double expectedSpeed = speed;
        if(speed > 0 && location() >= Constants.SHOULDER_UPPER_LIMIT){
            expectedSpeed = 0;
        }
        else if(speed < 0 && location() <= Constants.SHOULDER_LOWER_LIMIT){
            expectedSpeed = 0;
        }
        shoulderMotor.set(expectedSpeed);
    }
}
