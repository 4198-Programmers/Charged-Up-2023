package frc.robot;

public class Maths {

    public static double positionConversion(double encoderPos) { // converts position to meters for system use
        double circumferenceMeters = (Math.PI * (Constants.WHEEL_DIAMTER_METERS));
        double distanceMeters = (encoderPos / Constants.MOTOR_CONVERSION_FACTOR) * circumferenceMeters;
        // uses encoder to get 1-100% of a rotation, converting to meters
        return distanceMeters;
    }

    public static double[] swerveMath(double xAxis, double yAxis, double zAxis, int spinAngle) {

        /*
         * make more descriptive names when working (not now)
         */
        double x = xAxis;
        double y = yAxis;
        double z = zAxis;
        double angleAdjust = Math.toRadians(spinAngle);

        double mathX = x + (z * Math.cos(angleAdjust));
        double mathY = y + (z * Math.sin(angleAdjust));
        double angle = Math.toDegrees(Math.atan(mathX / mathY));
        double wantedAngle;
        double setSpeed;
        double wantedSpeed = Math.sqrt((x * x) + (y * y));

        if (wantedSpeed >= 1) { // to make sure the max speed is what we want
            wantedSpeed = 1;
        } else if (wantedSpeed <= -1) {
            wantedSpeed = -1;
        } else {
            // System.out.println("within bounds");
        }
        setSpeed = (wantedSpeed / 2) + (z / 2);

        if (xAxis / yAxis < 0) {
            angle -= 180;
        }
        if (xAxis > 0.05) {
            angle += 180;
        }
        if (angle < 0) {
            wantedAngle = angle + 360;
        } else if (angle >= 0) {
            wantedAngle = angle;
        } else {
            wantedAngle = 0;
            System.out.println("Something went wrong");
        }
        double[] outArr = new double[2];
        outArr[0] = wantedAngle;
        outArr[1] = (setSpeed * 0.25);
        return outArr;

    }
}
