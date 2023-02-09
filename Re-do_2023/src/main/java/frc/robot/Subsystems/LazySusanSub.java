package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LazySusanSub extends SubsystemBase{
    private final CANSparkMax susanMotor = new CANSparkMax(Constants.SUSAN_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder susanEncoder = susanMotor.getEncoder();

    public double getLocation(){
        return susanEncoder.getPosition();
    }

    public void spinSusan(double speed){
        double expectedSpeed = speed;
        if(getLocation() >= Constants.MAX_SUSAN_RIGHT_POSITION && speed > 0){
            expectedSpeed = 0;
        }
        else if(getLocation() <= Constants.MAX_SUSAN_LEFT_POSITION && speed < 0){
            expectedSpeed = 0;
        }
        susanMotor.set(expectedSpeed);
        System.out.println(getLocation());
    }

    public void stopSusan(){
        susanMotor.set(0);
    }
    
}
