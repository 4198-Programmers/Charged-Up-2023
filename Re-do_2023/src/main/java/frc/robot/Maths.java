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

    public static double DistanceFromTarget(double pitch) { // modified to be more accurate for testing
        double distanceToTarget = (2.25 / (Math.tan(pitch)));
        return distanceToTarget;
    }

    public static double distanceFromTargetBasedArea(double targetArea) {
        // calculates distance in inches from target to internal edge of the arm pocket
        // of the robot (the frame)
        double a = 123.929;
        double b = 0.253566;
        double m = 1.42053;
        double c = 33.1463;
        double distance = (a * (Math.pow(b, (m * targetArea)))) + c;
        return distance;

    }

    public static double distancePhotonSpeedCalc(double distanceDifference) {
        double a = -0.0000452121;
        double b = 0.00175731;
        double c = 0.0254966;
        double d = 0.0185907;
        double speed = (a * Math.pow(distanceDifference, 3)) + (b * Math.pow(distanceDifference, 2))
                + (c * distanceDifference) + d;
        return speed;
    }

    public static double translateAndSpinPhotonSpeedCalc(double difference) {
        double a = 0.00000857699;
        double b = -0.00136805;
        double c = 0.0673691;
        double d = -0.00127928;
        double speed = (a * Math.pow(difference, 3)) + (b * Math.pow(difference, 2)) + (c * difference) + d;

        return speed;
    }

    public static double distanceFromTargetRegression(double pitch) {
        double distanceToTarget = Math.pow(Constants.REGRESSION_A_VALUE, (pitch + Constants.REGRESSION_H_VALUE))
                + Constants.REGRESSION_K_VALUE;
        return distanceToTarget;
    }
}
