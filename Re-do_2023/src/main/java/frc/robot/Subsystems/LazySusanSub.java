package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Maths;

public class LazySusanSub extends SubsystemBase {
    private final CANSparkMax susanMotor;
    private final RelativeEncoder susanEncoder;
    int susanDirectionToggle = 1;
    public boolean susanDisable;
    // private final SparkMaxPIDController susanController;

    // private DigitalInput sensor = new
    // DigitalInput(Constants.SUSAN_SENSOR_CHANNEL);
    public LazySusanSub(boolean susanDisable) {
        this.susanDisable = susanDisable;
        if (!susanDisable) {
            susanMotor = new CANSparkMax(Constants.SUSAN_MOTOR_ID, MotorType.kBrushless);
            susanEncoder = susanMotor.getEncoder();
            // this.susanController = susanMotor.getPIDController();
            // susanController.setOutputRange(-1, 1);
        } else {
            susanMotor = null;
            susanEncoder = null;
            // this.susanController = null;
        }

    }

    public boolean getSensorValue() {
        // return sensor.get();
        return false;
    }

    public double getLocation() {
        if (susanDisable) {
            return 0;
        }
        return susanEncoder.getPosition();
    }

    public void zeroPosition() {
        if (susanDisable) {
            return;
        }
        susanEncoder.setPosition(0);
    }

    public void stopSusan() {
        if (susanDisable) {
            return;
        }
        susanMotor.set(0);
    }

    public void toggleSusan() {
        if (susanDisable) {
            return;
        }
        susanDirectionToggle *= -1;
    }

    public void mode(IdleMode mode) {
        if (susanDisable) {
            return;
        }
        susanMotor.setIdleMode(mode);
    }

    public double getRotation() {
        if (susanDisable) {
            return 0;
        }
        return Maths.arcLengthToRotations(susanEncoder.getPosition());
    }

    public void spinSusanWithAngles(double speed, double wantedDegrees) {
        // if (getLocation() - wantedDegrees < -0.5) {
        // susanMotor.set(-speed);
        // } else if (getLocation() - wantedDegrees > 0.5) {
        // susanMotor.set(speed);
        // }
        if (susanDisable) {
            return;
        }
        susanMotor.set(0);
    }

    public void spinSusan(double speed) { // counterclockwise = negative
        if (susanDisable) {
            return;
        }

        double expectedSpeed = speed * susanDirectionToggle;
        // System.out.println(getLocation() + "susan");

        // if (getLocation() >=
        // Maths.degreesToRotations_Susan(Constants.SUSAN_MAX_ANGLE) && speed > 0) {
        // expectedSpeed = 0;
        // } else if (getLocation() <=
        // Maths.degreesToRotations_Susan(-Constants.SUSAN_MAX_ANGLE) && speed < 0) {
        // expectedSpeed = 0;
        // }
        if (getSensorValue()) {
            zeroPosition();
        }

        susanMotor.set(expectedSpeed);
        // susanMotor.set(0);
    }

    // public void susanPIDSpin(double wantedPos){
    // susanController.setReference(wantedPos, CANSparkMax.ControlType.kPosition);
    // }

    public void susanEquationSpin(double wantedPos, double maxSpeed) {
        double differenceDistance = wantedPos - susanEncoder.getPosition();
        double speedMod = (Math.abs(differenceDistance) * 0.01333) + 0.025;// m on the linear curve
        susanMotor.set(speedMod * maxSpeed);
        System.out.println(speedMod * maxSpeed);
    }

    public void setSusanAngleCP(double wantedAngle) { // counterclockwise = negative
        if (susanDisable) {
            return;
        }
        // if (getLocation() < Maths.degreesToRotations_Susan(wantedAngle) -
        // Maths.degreesToRotations_Susan(5)) {
        // spinSusan(0.3);
        // } else if (getLocation() > Maths.degreesToRotations_Susan(wantedAngle) +
        // Maths.degreesToRotations_Susan(5)) {
        // spinSusan(-0.3);
        // } else {
        // spinSusan(0);
        // }
        susanMotor.set(0);
    }

    public double getSpeed() {
        return susanMotor.get();
    }
}
