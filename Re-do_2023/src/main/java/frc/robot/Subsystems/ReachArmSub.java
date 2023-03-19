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

    public ReachArmSub() {
        encoder.setDistancePerPulse((Constants.REACH_ENCODER_WHEEL_DIAMETER * Math.PI));
        reachMotor.setClosedLoopRampRate(0.5);
    }

    public void moveReach(double speed) {
        // System.out.println(encoder.getDistance() +"Reach");
        reachMotor.set(speed);
    }

    public void zeroEncoder() {
        encoder.reset();
    }

    public double getPosition() {
        return -encoder.getDistance();// enc is backwards
    }

    public double getSpeed() {
        return reachMotor.get();
    }

    public void stopReach() {
        reachMotor.set(0);
    }

    public void moveReachPosBased(double vertPos) {
        double reachWantedPos = 15907.1 * (Math.pow(0.5007, vertPos));

        if (getPosition() < (reachWantedPos + Constants.REACH_ENCODER_TOLERANCE)
                && getPosition() > (reachWantedPos - 15)) {
            moveReach(0);
        } else if (getPosition() < reachWantedPos - Constants.REACH_ENCODER_TOLERANCE) {
            moveReach(1);
        } else if (getPosition() > reachWantedPos + Constants.REACH_ENCODER_TOLERANCE) {
            moveReach(-1);
        }
    }

}
