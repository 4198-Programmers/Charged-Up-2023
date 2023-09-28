package frc.robot;

import frc.robot.SwerveDrive.SdsModuleConfigurations;

public final class Constants {
    public static final double DRIVE_TRAIN_WIDTH = 0;
    public static final double DRIVE_TRAIN_LENGTH = 0;
    //Swerve Constants
    /**More of a speed modifier than a max voltage */
    public static final double MAX_VOLTAGE = 12.0;
    /**5880 max rotations of the motor per second */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0
        * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
        * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    /*
     * Hypotenuse of the length/2 and width/2 is going to be the exact distance from the center of the robot to 
     * each module.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
        / Math.hypot(DRIVE_TRAIN_WIDTH / 2.0, DRIVE_TRAIN_LENGTH / 2.0);
    
}
