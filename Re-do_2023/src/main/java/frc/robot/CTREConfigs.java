package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class CTREConfigs{
    public CANCoderConfiguration coderConfiguration;

    public CTREConfigs(){
        coderConfiguration = new CANCoderConfiguration();
        coderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        coderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        coderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
