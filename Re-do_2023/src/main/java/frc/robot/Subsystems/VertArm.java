package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VertArm extends SubsystemBase {
    private final CANSparkMax verticalMotor = new CANSparkMax(Constants.VERTICAL_MOVER_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder verticalEncoder = verticalMotor.getEncoder();

    public double getLocation() {
        return verticalEncoder.getPosition();
    }

    public void ZeroArm() {
        verticalEncoder.setPosition(0);
    }

    public void moveArm(double speed) {
        // System.out.println(getLocation() + "vert");
        verticalMotor.set(speed);
    }

    public void vertEquationSpin(double wantedPos, double maxSpeed) {
        double differenceDistance = wantedPos - verticalEncoder.getPosition();
        double speedMod = (Math.abs(differenceDistance) * 0.4) + 0.25;// m on the linear curve
        verticalMotor.set(speedMod * maxSpeed);
        System.out.println(speedMod * maxSpeed);
    }

    public double getSpeed() {
        return verticalMotor.get();
    }

    public double getPosition() {
        return verticalEncoder.getPosition();
    }

    public void stopArm() {
        verticalMotor.set(Constants.VERT_ARM_NO_DROP_SPEED);
    }

}
