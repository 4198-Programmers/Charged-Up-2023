package frc.robot;

public class Maths {
    public static double arcLengthToRotations(double arcLength) {
        double angle = (arcLength / Constants.SUSAN_CIRCUMFERENCE) * 360;
        return angle;
    }
    public static double DistanceFromTarget(double pitch) {
        double distanceToTarget = Constants.CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE/Math.tan(Math.toRadians(pitch));
        return distanceToTarget;
    }
    public static double degreesToRotations_Susan(double wantedDegrees){
        return wantedDegrees;
    }
}
