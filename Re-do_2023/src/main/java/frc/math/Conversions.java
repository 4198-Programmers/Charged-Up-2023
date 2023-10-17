package frc.math;

public class Conversions {
    
    /**
     * 
     * @param counts Canspark Counts
     * @param gearRatio Gear ration between Canspark and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double cansparkToDegress(double counts, double gearRatio){
        return counts * (360.0 / (gearRatio * 5880.0));   //max of 5880 rotations per second
    }

    /**
     * 
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Canspark and Mechanism
     * @return Canspark Counts
     */
    public static double degreesToCanspark(double degrees, double gearRatio){
        double ticks = degrees / (360.0 / (gearRatio * 5880.0));
        return ticks;
    }

    /**
     * @param velocityCounts canspark velocity counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return RPM of Mechanism
     */
    public static double cansparkToRPM(double velocityCounts, double gearRatio){
        double motorRPM = velocityCounts * (600.0 / 5880.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between canspark and Mechanism
     * @return RPM of Mechanism
     */
    public static double RPMToCanspark(double RPM, double gearRatio){
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (5880.0 / 600.0); //600 because that is 60 seconds in tenths of a second.
        return sensorCounts;
    }

    /**
     * 
     * @param velocityCounts Canspark velocity counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Canspark and Mechanism
     * @return Cansparkmax Velocity Counts
     */
    public static double cansparkToMPS(double velocityCounts, double circumference, double gearRatio){
        double wheelRPM = cansparkToRPM(velocityCounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * 
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between canspark and mechanism
     * @return Canspark Velocity counts
     */
    public static double MPSToCanspark(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToCanspark(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Canspark Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Canspark and Wheel
     * @return meters
     */
    public static double cansparkToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 5880.0));
    }

    /**
     * 
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Canspark and Wheel
     * @return Canspark Position Counts
     */
    public static double MetersToCanspark(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * 5880.0));
    }
    
}