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
        
    public static double DistanceFromTarget(double pitch) {
        double distanceToTarget = Constants.CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE/Math.tan(Math.toRadians(pitch));
        return distanceToTarget;
    }

    public static double distanceFromTargetRegression(double pitch) {
        double distanceToTarget = Math.pow(Constants.REGRESSION_A_VALUE, (pitch + Constants.REGRESSION_H_VALUE)) + Constants.REGRESSION_K_VALUE;
        return distanceToTarget;
    }
}
