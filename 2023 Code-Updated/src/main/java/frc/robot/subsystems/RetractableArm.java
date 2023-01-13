package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RetractableArm extends SubsystemBase{
    CANSparkMax retractableMotor = new CANSparkMax(Constants.RETRACTABLE_ARM_MOTOR, MotorType.kBrushless);

    public void setSpeed(double speed){
        retractableMotor.set(speed);
    }
}