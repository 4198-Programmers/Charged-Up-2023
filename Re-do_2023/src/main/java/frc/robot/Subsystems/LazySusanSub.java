package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Maths;

public class LazySusanSub extends SubsystemBase {
    private final CANSparkMax susanMotor = new CANSparkMax(Constants.SUSAN_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder susanEncoder = susanMotor.getEncoder();

    public double getLocation() {
        return susanEncoder.getPosition();
    }

    public void spinSusan(double speed) {
        double expectedSpeed = speed;
        if (getLocation() >= Constants.MAX_SUSAN_RIGHT_POSITION && speed > 0) { // counterclockwise = negative
            expectedSpeed = 0;
        } else if (getLocation() <= Constants.MAX_SUSAN_LEFT_POSITION && speed < 0) {
            expectedSpeed = 0;
        }
        susanMotor.set(expectedSpeed);
    }

    public void stopSusan() {
        susanMotor.set(0);
    }

    public void mode(IdleMode mode) {
        susanMotor.setIdleMode(mode);
    }

    public double getRotation() {
        return Maths.arcLengthToRotations(susanEncoder.getPosition());
    }

    public void spinSusanWithAngles(double speed, double wantedDegrees) {
        if (getLocation() - wantedDegrees < -0.5) {
            susanMotor.set(-speed);
        } else if (getLocation() - wantedDegrees > 0.5) {
            susanMotor.set(speed);
        }
    }

    public void spinSusanCP(double speed) { // counterclockwise = negative

        double expectedSpeed = speed;

        if (getLocation() >= Maths.degreesToRotations_Susan(Constants.SUSAN_MAX_ANGLE) && speed > 0) {
            expectedSpeed = 0;
        } else if (getLocation() <= Maths.degreesToRotations_Susan(-Constants.SUSAN_MAX_ANGLE) && speed < 0) {
            expectedSpeed = 0;
        }

        susanMotor.set(expectedSpeed);
    }

    public void setSusanAngleCP(double wantedAngle) { // counterclockwise = negative
        if (getLocation() < Maths.degreesToRotations_Susan(wantedAngle) - Maths.degreesToRotations_Susan(5)) {
            spinSusanCP(0.3);
        } else if (getLocation() > Maths.degreesToRotations_Susan(wantedAngle) + Maths.degreesToRotations_Susan(5)) {
            spinSusanCP(-0.3);
        } else {
            spinSusanCP(0);
        }
    }
}