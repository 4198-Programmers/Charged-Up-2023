package frc.robot.SwerveDrive;
/**Configures the module to match what we are using for the wheels. */
public class SdsModuleConfigurations {
    
    public static final ModuleConfiguration MK3_STANDARD = new ModuleConfiguration(
        0.1016, 
        (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0), 
        true, 
        (15.0 / 32.0) * (10.0 / 60.0), 
        true);
/**This is the module that we use in our robot. */
    public static final ModuleConfiguration MK4I_L2 = new ModuleConfiguration(
        0.10033, 
        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), 
        true, 
        (14.0 / 50.0) * (10.0 / 60.0), 
        false);

    private SdsModuleConfigurations(){
        
    }
}
