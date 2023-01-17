package frc.robot;

public class Maths {

    public static double positionConversion(double encoderPos) { //converts position to meters for system use
        double circumferenceMeters = (Math.PI * (Constants.WHEEL_DIAMTER_METERS));
        double distanceMeters = (encoderPos/Constants.MOTOR_CONVERSION_FACTOR) * circumferenceMeters; //uses encoder to get 1-100% of a rotation, converting to meters
        return distanceMeters;
    }
    
}
