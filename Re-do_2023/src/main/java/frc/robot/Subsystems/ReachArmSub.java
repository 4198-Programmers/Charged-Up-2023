package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachArmSub extends SubsystemBase{
    private final CANSparkMax reachMotor = new CANSparkMax(Constants.IN_OUT_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder reachEncoder = reachMotor.getEncoder();

    public double getPosition(){
        return reachEncoder.getPosition();
    }

    public void moveReach(double speed){
        double expectedSpeed = speed;
        if(getPosition() >= Constants.MAX_REACH && speed > 0){
            expectedSpeed = 0;
        }
        else if(getPosition() <= Constants.MIN_REACH && speed < 0){
            expectedSpeed = 0;
        }
        reachMotor.set(expectedSpeed);
    }

    public void stopReach(){
        reachMotor.set(0);
    }
    
}
