package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachArmSub extends SubsystemBase{
    private final CANSparkMax reachMotor = new CANSparkMax(Constants.IN_OUT_MOTOR_ID, MotorType.kBrushless);

    public void moveReach(double speed){
        reachMotor.set(speed);
    }

    public void stopReach(){
        reachMotor.set(0);
    }
    
}
