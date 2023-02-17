package frc.robot;

public class Maths {
    public static double arcLengthToRotations(double arcLength) {
        double angle = (arcLength / Constants.SUSAN_CIRCUMFERENCE) * 360;
        return angle;
    }

    public static double degreesToRotations_Susan(double wantedDegrees) {
        double degreesPerRotate = 360 / Constants.MOTOR_ROTATIONS_PER_360_SUSAN;
        return wantedDegrees / degreesPerRotate;
    }
}
