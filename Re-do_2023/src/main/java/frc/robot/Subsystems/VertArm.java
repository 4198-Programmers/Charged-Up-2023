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

    public void ZeroArm(){
        verticalEncoder.setPosition(0);
    }

    public void moveArm(double speed) {
        double expectedSpeed = speed;
        verticalMotor.set(expectedSpeed);
    }

    public double getSpeed() {
        return verticalMotor.get();
    }

    public void stopArm() {
        verticalMotor.set(Constants.VERT_ARM_NO_DROP_SPEED);
    }
    public void autoVert(double speed, double wantedDistance) {
        if (getLocation() - wantedDistance < -Constants.VERT_OFFSET) {
            verticalMotor.set(-speed);
        } else if (getLocation() - wantedDistance > Constants.VERT_OFFSET) {
            verticalMotor.set(speed);
        }
    }
}
