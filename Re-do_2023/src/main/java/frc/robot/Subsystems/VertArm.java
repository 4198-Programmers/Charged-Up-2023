package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VertArm extends SubsystemBase{
    private final CANSparkMax verticalMotor = new CANSparkMax(Constants.VERTICAL_MOVER_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder verticalEncoder = verticalMotor.getEncoder();

    public double getLocation(){
        return verticalEncoder.getPosition();
    }
    
    public void moveArm(double speed){
        double expectedSpeed = speed;
        if(getLocation() >= Constants.MAX_VERTICAL_POSITION && speed > 0){
            expectedSpeed = 0;
        }
        else if(getLocation() <= Constants.MIN_VERTICAL_POSITION && speed < 0){
            expectedSpeed = 0;
        }
        verticalMotor.set(expectedSpeed);
    }

    public void stopArm(){
        verticalMotor.set(0);
    }
}
