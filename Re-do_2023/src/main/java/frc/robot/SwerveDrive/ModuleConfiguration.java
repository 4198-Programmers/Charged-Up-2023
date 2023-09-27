package frc.robot.SwerveDrive;

public class ModuleConfiguration {
    private final double wheelDiameter;
    private final double driveReduction;
    private final boolean driveInverted;

    private final double steerReduction;
    private final boolean steerInverted;

    /**
     * 
     * Creates a new module configuration.
     * 
     * @param wheelDiameter The diameter of the module's wheel in meters.
     * @param driveReduction The overall drive reduction of the module. Multipllying motor rotations by this value should result in wheel rotations.
     * @param driveInverted Whether the drive motor should be inverted. If there is an odd number of gea reductions this is typically true.
     * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value should result in rotations of the steering pulley.
     * @param steerInverted Whether the steer motor should be inverted. If there is an odd number of gear reductions this is typically true.
     * 
     */
    
public ModuleConfiguration(double wheelDiameter, double driveReduction, boolean driveInverted, 
                            double steerReduction, boolean steerInverted){
    this.wheelDiameter = wheelDiameter;
    this.driveReduction = driveReduction;
    this.driveInverted = driveInverted;
    this.steerReduction = steerReduction;
    this.steerInverted = steerInverted;
}
/**
 * Gets the diameter of the wheel in meters
 * @return wheelDiameter
 */
public double getWheelDiameter(){
    return wheelDiameter;
}
/**Gets teh overall reduction of the drive system.
 * <p>
 * if this value is multiplied by drive motor rotations, the result would be drive wheel rotations.
 * @return driveReduction
 */
public double getDriveReduction(){
    return driveReduction;
}

/**
 * Gets if the drive motor should be inverted 
 * @return driveInverted
 */
public boolean isDriveInverted(){
    return driveInverted;
}

/**
 * Gets the overall reduction of the steer system.
 * <p>
 * If this value is multiplied by steering motor rotations the result would be steerig pulley rotations.
 * @return steerReduction
 */
public double getSteerReduction(){
    return steerReduction;
}

/**
 * Gets if the steering motor should be inverted
 * @return steerInverted
 */
public boolean isSteerInverted(){
    return steerInverted;
}

/*
 * Indicates whether some other object is "equal to" this one.
 */
@Override
public boolean equals(Object o){
    if(this == o) return true;
    if(o == null || getClass() != o.getClass()) return false;

    ModuleConfiguration that = (ModuleConfiguration) o;
    return Double.compare(that.getWheelDiameter(), getWheelDiameter()) == 0 &&
}
}
