package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax intake = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);


    public void SetSpeed(double speed) {
        intake.set(speed);
    }

}
