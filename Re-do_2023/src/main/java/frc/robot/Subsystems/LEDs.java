package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    // private final CANdle lights = new CANdle(Constants.LED_CAN_ID);
    // private final CANdleConfiguration configLight = new CANdleConfiguration();
    AddressableLED leds = new AddressableLED(0);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(15);

    public void startLEDS() {
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void redLED() {
        // for (var i = 0; i < buffer.getLength(); i++) {
        //     buffer.setRGB(i, 255, 0, 0);
        // }
        buffer.setRGB(1, 255, 0, 0);
        buffer.setRGB(2, 255, 0, 0);
        buffer.setRGB(3, 255, 0, 0);
        buffer.setRGB(4, 255, 0, 0);
        buffer.setRGB(5, 255, 0, 0);
        buffer.setRGB(6, 255, 0, 0);
        buffer.setRGB(7, 255, 0, 0);
        buffer.setRGB(8, 255, 0, 0);
        buffer.setRGB(9, 255, 0, 0);
        buffer.setRGB(10, 255, 0, 0);
        // buffer.setRGB(11, 255, 0, 0);
        // buffer.setRGB(12, 255, 0, 0);
        // buffer.setRGB(13, 255, 0, 0);
        // buffer.setRGB(14, 255, 0, 0);
        // buffer.setRGB(15, 255, 0, 0);
        leds.start();
        leds.setData(buffer);
        System.out.println("running leds");
    }

}