package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VertArm extends SubsystemBase{
    private final CANSparkMax verticalMotor = new CANSparkMax(Constants.VERTICAL_MOVER_MOTOR_ID, MotorType.kBrushless);
    
    public void moveArm(double speed){
        verticalMotor.set(speed);
    }

    public void stopArm(){
        verticalMotor.set(0);
    }
}
