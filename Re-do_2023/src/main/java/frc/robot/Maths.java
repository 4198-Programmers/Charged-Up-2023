package frc.robot;

public class Maths {
    public static double arcLengthToRotations(double arcLength) {
        double angle = (arcLength / Constants.SUSAN_CIRCUMFERENCE) * 360;
        return angle;
    }

    public static double degreesToRotations_Susan(double wantedDegrees){

    }
}
