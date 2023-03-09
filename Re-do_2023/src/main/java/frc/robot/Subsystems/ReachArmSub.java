package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachArmSub extends SubsystemBase {
    private final CANSparkMax reachMotor = new CANSparkMax(Constants.IN_OUT_MOTOR_ID, MotorType.kBrushed);
    private Encoder encoder = new Encoder(0, 1, false, EncodingType.k1X);

    public void moveReach(double speed){
        double expectedSpeed = speed;
        // if(getLocation() >= Constants.MAX_REACH && speed > 0){
        // expectedSpeed = 0;
        // }
        // else if(getLocation() <= Constants.MIN_REACH && speed < 0){
        // expectedSpeed = 0;
        // }
        reachMotor.set(expectedSpeed);
        // System.out.println("Reach " + getLocation());
    }

    public double getSpeed() {
        return reachMotor.get();
    }

    public void stopReach() {
        reachMotor.set(0);
    }

}
