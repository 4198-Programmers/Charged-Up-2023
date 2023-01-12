package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase{
    CANSparkMax motor;
    public void getMotor(double motorNumber){
        motor = new CANSparkMax(motorNumber, MotorType.kBrushless);
    }
    public void setMotor(double speed){
        motor.set(speed);
    }
}
