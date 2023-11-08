package frc.robot.AttemptOne;

import frc.robot.Constants;

public class Conversions {
    private static double maxNumberOfRotations = Constants.MAX_NUMBER_OF_ROTATIONS;
    private static double angleGearRatio = Constants.ANGLE_GEAR_RATIO;
    private static double driveGearRatio = Constants.DRIVE_GEAR_RATIO;
    /**
     * @param counts Neo Counts
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double neoToDegrees(double counts) {
        return counts * (360.0 / (angleGearRatio * maxNumberOfRotations));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between neo and Mechanism
     * @return Neo Counts
     */
    public static double degreesToNeo(double degrees) {
        double ticks =  degrees / (360.0 / (angleGearRatio * maxNumberOfRotations));
        return ticks;
    }

    /**
     * @param velocityCounts Neo Velocity Counts
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return RPM of Mechanism
     */
    public static double neoToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / maxNumberOfRotations);       //600 because it is for every 10th of a second 
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToNeo(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Neo Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return Neo Velocity Counts
     */
    public static double neoToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = neoToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return Neo Velocity Counts
     */
    public static double MPSToNeo(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToNeo(wheelRPM, gearRatio);
        return wheelVelocity;
    }

        /**
     * @param positionCounts Neo Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Meters
     */
    public static double neoToMeters(double positionCounts, double circumference){
        return positionCounts * (circumference / (driveGearRatio * maxNumberOfRotations));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Neo Position Counts
     */
    public static double MetersToNeo(double meters, double circumference){
        return meters / (circumference / (driveGearRatio * maxNumberOfRotations));
    }
}
