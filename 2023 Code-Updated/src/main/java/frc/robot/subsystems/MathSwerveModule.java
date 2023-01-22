package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MathSwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax spinMotor;
    CANCoder spinEncoder;
    byte spinOptimize;

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
        double conditionArr[] = { 10, 5, 3, 2, 1 };
        double speedArr[] = { 0.3, 0.2, 0.1, 0.05, 0.02 };
        double speedToGive;
        double conditionAmount;
        double driveSpeed = speed;
        spinPos = spinEncoder.getAbsolutePosition();

        if (Math.abs(spinPos - wantedAngle) > 90 && wantedAngle > spinPos) {
            spinOptimize = -1;
            angleToGive = Math.abs(wantedAngle - 180);
        } else if (Math.abs(spinPos - wantedAngle) > 90 && wantedAngle < spinPos) {
            spinOptimize = -1;
            angleToGive = Math.abs(wantedAngle + 180);
        } else if (Math.abs(spinPos - wantedAngle) < 90) {
            angleToGive = wantedAngle;
            spinOptimize = 1;
        } else {
            spinOptimize = 1;
            angleToGive = 0;
        }

        for (int i = 0; i < 5; i++) {
            speedToGive = 0.05;
            conditionAmount = conditionArr[i];
            if (spinPos > angleToGive + conditionAmount && angleToGive < 361) {
                spinMotor.set(speedToGive);
            } else if (spinPos < angleToGive - conditionAmount && angleToGive < 361) {
                spinMotor.set(-speedToGive);
            } else {
                spinMotor.set(0);
            }
        }
        System.out.println(angleToGive + " wantedAngle");
        driveMotor.set(driveSpeed * spinOptimize);
    }

}
