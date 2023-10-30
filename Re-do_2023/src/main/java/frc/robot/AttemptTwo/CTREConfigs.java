package frc.robot.AttemptTwo;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class CTREConfigs {
    public static CANCoderConfiguration canCoderConfig;
    public CTREConfigs(){
        canCoderConfig = new CANCoderConfiguration();

        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.sensorDirection = false;
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
