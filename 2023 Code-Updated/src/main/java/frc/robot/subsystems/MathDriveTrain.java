package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Maths;

public class MathDriveTrain extends SubsystemBase {
    MathSwerveModule frModule = new MathSwerveModule(4, 3, 10, -17.9296875);
    MathSwerveModule flModule = new MathSwerveModule(2, 1, 9, -87.890625);
    MathSwerveModule brModule = new MathSwerveModule(8, 5, 11, 27.94921875);// -332.05078125
    MathSwerveModule blModule = new MathSwerveModule(6, 7, 12, -291.181640625);
    // private final MathSwerveModule frModule = new MathSwerveModule(4, 3, 10,
    // -15.01171875); // +27 -42.01171875
    // private final MathSwerveModule flModule = new MathSwerveModule(2, 1, 9,
    // 89.12890625);// +32 57.12890625
    // private final MathSwerveModule brModule = new MathSwerveModule(8, 5, 11,
    // 30.24804688);// +33.5 -3.251953125
    // private final MathSwerveModule blModule = new MathSwerveModule(6, 7, 12,
    // 70.149921875);// +35 35.149921875
    double calculatedAngle;

    public void drive(double xAxisArg, double yAxisArg, double zAxisArg) {
        double xAxis = xAxisArg;
        double yAxis = yAxisArg;
        double zAxis = zAxisArg;
        byte xFlip;
        double zSpeed = 0.15 * zAxis;
        // double atanAngle = Math.atan(xAxis / yAxis);
        double[] frSettings = Maths.swerveMath(xAxis, yAxis, zSpeed, 45);
        double[] flSettings = Maths.swerveMath(xAxis, yAxis, zSpeed, 135);
        double[] blSettings = Maths.swerveMath(xAxis, yAxis, zSpeed, 225);// 135 45 225 315
        double[] brSettings = Maths.swerveMath(xAxis, yAxis, zSpeed, 315);


        // if (Math.toDegrees(atanAngle) < 0 && zAxis >= -0.05 && zAxis <= 0.05) {
        // calculatedAngle = Math.toDegrees(atanAngle) + 360c;
        // } else if (zAxis >= -0.05 && zAxis <= 0.05) {
        // calculatedAngle = Math.toDegrees(atanAngle);
        // }

        if (xAxis > 0) {
            xFlip = -1;
        } else {
            xFlip = 1;
        }

        // if (zAxis < -0.05 || zAxis > 0.05) {
        // frModule.setDesiredState(135, zSpeed);
        // flModule.setDesiredState(45, zSpeed);
        // brModule.setDesiredState(225, zSpeed);
        // blModule.setDesiredState(315, zSpeed);
        // } else
        if (xAxis < -0.05 || xAxis > 0.05 || yAxis < -0.05 || yAxis > 0.05 || zAxis < -0.05 || zAxis > 0.05) {
            frModule.setDesiredState(frSettings[0], frSettings[1]);
            flModule.setDesiredState(flSettings[0], flSettings[1]);
            brModule.setDesiredState(brSettings[0], brSettings[1]);
            blModule.setDesiredState(blSettings[0], blSettings[1]);
        } else {
            frModule.setDesiredState(700, 0);
            flModule.setDesiredState(700, 0);
            brModule.setDesiredState(700, 0);
            blModule.setDesiredState(700, 0);
        }
    }

    public void AutoDrive(double angleArg, double speedArg){
        frModule.setDesiredState(angleArg, speedArg);
        flModule.setDesiredState(angleArg, speedArg);
        brModule.setDesiredState(angleArg, speedArg);
        blModule.setDesiredState(angleArg, speedArg);
    }

}
