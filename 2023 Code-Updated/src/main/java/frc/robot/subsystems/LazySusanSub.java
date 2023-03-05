package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LazySusanSub extends SubsystemBase {
    
    TalonSRX lazySusanMotor = new TalonSRX(2);

    public void spinSusan(double speed) {
        lazySusanMotor.set(TalonSRXControlMode.PercentOutput, speed); // will read the same [-1,1] as CANSparks
    }

    public void stopSusan() {
        lazySusanMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

}
