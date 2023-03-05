package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VertArm extends SubsystemBase {

    CANSparkMax verticalExtenderMotor = new CANSparkMax(1, MotorType.kBrushless);

    public void moveArmUp(double speed) {
        verticalExtenderMotor.set(speed); // will read the same [-1,1] as CANSparks
    }

    public void stopArm() {
        verticalExtenderMotor.set(0);
    }

}
