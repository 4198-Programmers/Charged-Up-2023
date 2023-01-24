package frc.robot;

public class Maths {
    public static double DistanceFromTarget(double pitch) {
        double distanceToTarget = Constants.CAMERA_TO_APRILTAG_HEIGHT_DIFFERENCE/Math.tan(pitch);
        return distanceToTarget;
    }
}
