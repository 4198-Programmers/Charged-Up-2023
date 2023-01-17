package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase{
    CANSparkMax elbowMotor = new CANSparkMax(Constants.ELBOW_MOTOR, MotorType.kBrushless);
    RelativeEncoder elbowEncoder = elbowMotor.getEncoder();

    public void setSpeed(double speed){
        elbowMotor.set(speed);
    }
    public double getLocation(){
        return elbowEncoder.getPosition();
    }
    public void setSpeedWithLimits(double speed){
        double expectedSpeed = speed;
        if(speed > 0 && getLocation() > Constants.ELBOW_UPPER_LIMIT){
            expectedSpeed = 0;
        }
        else if(speed < 0 && getLocation() < Constants.ELBOW_LOWER_LIMIT){
            expectedSpeed = 0;
        }
        elbowMotor.set(expectedSpeed);
    }
}