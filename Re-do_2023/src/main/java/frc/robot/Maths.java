package frc.robot;

public class Maths {
    public static double arcLengthToRotations(double arcLength) {
        double angle = (arcLength / Constants.SUSAN_CIRCUMFERENCE) * 360;
        if (angle > 180) {
            angle = angle - 360;
        }
        return angle;
    }
}
