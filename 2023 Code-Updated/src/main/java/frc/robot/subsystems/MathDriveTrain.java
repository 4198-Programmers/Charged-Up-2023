package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MathDriveTrain extends SubsystemBase {
    MathSwerveModule frModule = new MathSwerveModule(4, 3, 10, -17.9296875);
    MathSwerveModule flModule = new MathSwerveModule(2, 1, 9, 92.109375);
    MathSwerveModule brModule = new MathSwerveModule(8, 5, 11, -332.05078125);
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
        double speedX = 0.05 * xAxis;
        double speedY = 0.05 + yAxis;
        double speed = Math.sqrt((speedX * speedX) + (speedY * speedY));
        byte yFlip;
        double zSpeed = Math.abs(0.3 * zAxis);
        double atanAngle = Math.atan(xAxis / yAxis);
        byte zFlip;

        if (Math.toDegrees(atanAngle) < 0 && zAxis >= -0.05 && zAxis <= 0.05) {
            calculatedAngle = Math.toDegrees(atanAngle) + 90;
        } else if (zAxis >= -0.05 && zAxis <= 0.05) {
            calculatedAngle = Math.toDegrees(atanAngle);
        }

        if (speed >= 0.05) {
            speed = 0.05;
        } else if (speed <= -0.05) {
            speed = -0.05;
        }

        if (zAxis > 0) {
            zFlip = -1;
        } else {
            zFlip = 1;
        }

        if (yAxis < 0) {
            yFlip = -1;
        } else {
            yFlip = 1;
        }

        if (zAxis < -0.05 || zAxis > 0.05) {
            frModule.setDesiredState(135, -(zSpeed * zFlip));
            flModule.setDesiredState(45, (zSpeed * zFlip));
            brModule.setDesiredState(225, -(zSpeed * zFlip));
            blModule.setDesiredState(315, -(zSpeed * zFlip));
        } else if (xAxis < -0.05 || xAxis > 0.05 || yAxis < -0.05 || yAxis > 0.05) {
            frModule.setDesiredState(calculatedAngle, -(speed * yFlip));
            flModule.setDesiredState(calculatedAngle, (speed * yFlip));
            brModule.setDesiredState(calculatedAngle, -(speed * yFlip));
            blModule.setDesiredState(calculatedAngle, -(speed * yFlip));
        } else {
            frModule.setDesiredState(361, 0);
            flModule.setDesiredState(361, 0);
            brModule.setDesiredState(361, 0);
            blModule.setDesiredState(361, 0);
        }
    }

}
