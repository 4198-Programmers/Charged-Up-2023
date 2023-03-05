/*package frc.robot.Subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    private final CANdle lights = new CANdle(Constants.LED_CAN_ID);
    private final CANdleConfiguration configLight = new CANdleConfiguration();

    public LEDs(double brightness) { // brightness is 0-1 reflecting 0-100%
        configLight.stripType = LEDStripType.RGB;
        configLight.brightnessScalar = brightness;
        lights.configAllSettings(configLight);
    }

    public void setColor(int R, int G, int B) {
        lights.setLEDs(R, G, B);
    }

    public void changeBrightness(double brightness) {
        configLight.brightnessScalar = brightness;
    }

    public void rainbowLights(double brightness, double speed, int numberOfLights) {
        RainbowAnimation rainbow = new RainbowAnimation(brightness, speed, numberOfLights);
        lights.animate(rainbow);
    }
}
**/