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
    int susanDirectionToggle = 1;

    public double getLocation() {
        return susanEncoder.getPosition();
    }

    public void zeroPosition(){
        susanEncoder.setPosition(0);
    }

    public void stopSusan() {
        susanMotor.set(0);
    }

    public void toggleSusan(){
        susanDirectionToggle *= -1;
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

    public void spinSusan(double speed) { // counterclockwise = negative

        double expectedSpeed = speed * susanDirectionToggle;

        // if (getLocation() >= Maths.degreesToRotations_Susan(Constants.SUSAN_MAX_ANGLE) && speed > 0) {
        //     expectedSpeed = 0;
        // } else if (getLocation() <= Maths.degreesToRotations_Susan(-Constants.SUSAN_MAX_ANGLE) && speed < 0) {
        //     expectedSpeed = 0;
        // }

        susanMotor.set(expectedSpeed);
    }

    public void setSusanAngleCP(double wantedAngle) { // counterclockwise = negative
        if (getLocation() < Maths.degreesToRotations_Susan(wantedAngle) - Maths.degreesToRotations_Susan(5)) {
            spinSusan(0.3);
        } else if (getLocation() > Maths.degreesToRotations_Susan(wantedAngle) + Maths.degreesToRotations_Susan(5)) {
            spinSusan(-0.3);
        } else {
            spinSusan(0);
        }
    }
}
