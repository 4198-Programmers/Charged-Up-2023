package frc.robot.MathDriveFiles;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MathSwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax spinMotor;
    CANCoder spinEncoder;
    byte spinOptimize;
    byte i = 0;

    public MathSwerveModule(int driveMotorID, int spinMotorID, int CANcoderID, double abosluteOffset) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorID, MotorType.kBrushless);
        spinEncoder = new CANCoder(CANcoderID);
        spinEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // spinEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        spinEncoder.configMagnetOffset(abosluteOffset);
    }

    public void setDesiredState(double angle, double speed) {
        double spinPos;
        double wantedAngle = angle;
        double angleToGive;
        double conditionArr[] = { 20, 15, 10, 5 };
        double speedArr[] = { 0.2, 0.15, 0.1, 0.05 };
        double speedToGive;
        double conditionAmount;
        double driveSpeed = speed;
        spinPos = spinEncoder.getAbsolutePosition();
        double absoluteDistanceBetween = Math.abs(spinPos - wantedAngle);
        double speedScalar = Math.abs(wantedAngle / spinPos);
        if (i > 3) {
            i = 0;
        } else {
        }

        if (absoluteDistanceBetween > 90 && wantedAngle > spinPos) {
            spinOptimize = -1;
            angleToGive = wantedAngle - 180;

            if (angleToGive < 0) {
                angleToGive += 360;
            } else if (angleToGive >= 360) {
                angleToGive -= 360;
            } else {
                // System.out.println("in bounds");
            }

        } else if (absoluteDistanceBetween > 90 && wantedAngle < spinPos) {
            spinOptimize = -1;
            angleToGive = wantedAngle + 180;

            if (angleToGive < 0) {
                angleToGive += 360;
            } else if (angleToGive >= 360) {
                angleToGive -= 360;
            } else {
            }

        } else if (absoluteDistanceBetween < 90) {
            angleToGive = wantedAngle;
            spinOptimize = 1;
        } else {
            spinOptimize = 1;
            angleToGive = 0;
        }

        // speedToGive = speedArr[i];
        // conditionAmount = conditionArr[i];
        if (spinPos > angleToGive + 10) {// + conditionAmount
            spinMotor.set(0.15);
        } else if (spinPos < angleToGive - 10) {// - conditionAmount
            spinMotor.set(-0.15);
        } else if (spinPos > angleToGive + 5) {
            spinMotor.set(0.05);
        } else if (spinPos < angleToGive - 5) {
            spinMotor.set(-0.05);
        } else {
            spinMotor.set(0);
        }
        // else if (spinPos > angleToGive - conditionAmount && spinPos < angleToGive +
        // conditionAmount) {
        // i++;
        // }
        System.out.println(absoluteDistanceBetween);
        System.out.println(speedScalar * 0.2);
        System.out.println(i);
        driveMotor.set(driveSpeed * spinOptimize);
    }

}
