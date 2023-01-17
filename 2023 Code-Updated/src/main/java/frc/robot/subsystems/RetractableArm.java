package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RetractableArm extends SubsystemBase{
    CANSparkMax retractableMotor = new CANSparkMax(Constants.RETRACTABLE_ARM_MOTOR, MotorType.kBrushless);
    RelativeEncoder retractableEncoder = retractableMotor.getEncoder();
    public double getLocation(){
        return retractableEncoder.getPosition();
    }
    public void setSpeed(double speed){
        retractableMotor.set(speed);
    }
    public void setSpeedWithLimits(double speed){
        double expectedSpeed = speed;
        if(speed > 0 && getLocation() >= Constants.ARM_UPPER_LIMIT){
            expectedSpeed = 0;
        }
        else if(speed < 0 && getLocation() <= Constants.ARM_LOWER_LIMIT){
            expectedSpeed = 0;
        }
        
        retractableMotor.set(expectedSpeed);
    }
}