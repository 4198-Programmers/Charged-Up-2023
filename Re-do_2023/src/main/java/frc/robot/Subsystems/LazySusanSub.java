package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LazySusanSub extends SubsystemBase{
    private final CANSparkMax susanMotor = new CANSparkMax(Constants.SUSAN_MOTOR_ID, MotorType.kBrushless);

    public void spinSusan(double speed){
        susanMotor.set(speed);
    }

    public void stopSusan(){
        susanMotor.set(0);
    }
    
}
