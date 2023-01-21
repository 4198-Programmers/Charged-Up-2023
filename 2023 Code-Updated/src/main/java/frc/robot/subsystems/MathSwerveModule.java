package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MathSwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax spinMotor;
    CANCoder spinEncoder;

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
        byte spinOptimize;
        double wantedAngle = angle;
        double conditionArr[] = { 15, 10, 6, 3, 1 };
        double speedArr[] = { 0.14, 0.10, 0.08, 0.05, 0.02 };
        double speedToGive;
        double conditionAmount;
        spinPos = spinEncoder.getAbsolutePosition();

        if (Math.abs(spinPos - wantedAngle) > 90) {
            spinOptimize = -1;
        } else {
            spinOptimize = 1;
        }

        for (int i = 0; i < 5; i++) {
            speedToGive = speedArr[i];
            conditionAmount = conditionArr[i];
            if (spinPos > wantedAngle + conditionAmount && wantedAngle < 361) {
                spinMotor.set(speedToGive * spinOptimize);
            } else if (spinPos < wantedAngle - conditionAmount && wantedAngle < 361) {
                spinMotor.set(-speedToGive * spinOptimize);
            } else {
                spinMotor.set(0);
            }
        }
        System.out.println(wantedAngle + "watnedAngle");
        driveMotor.set(speed);
    }

}
