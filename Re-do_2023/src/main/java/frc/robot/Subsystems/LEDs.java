// package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class LEDs extends SubsystemBase {

//     private AddressableLED lights;
//     private AddressableLEDBuffer LEDBuffer;
//     public LEDs(int LEDCount){
//         lights = new AddressableLED(Constants.LED_CAN_ID);
//         LEDBuffer = new AddressableLEDBuffer(LEDCount);
//         lights.setLength(LEDCount);
//     }

//     public void dataSetter(){
//         lights.setData(LEDBuffer);
//         lights.start();
//     }

//     public void setOneLightRGB(int index, int R, int G, int B){
//         LEDBuffer.setRGB(index, R, G, B);
//     }
//     public void setLights(int start, int end, int R, int G, int B){
//         for(int i = start; i < end; i++){
//             setOneLightRGB(i, R, G, B);
//         }
//     }
//     public void turnLightsOff(){
//         for(int i = 0; i < LEDBuffer.getLength(); i++){
//             setOneLightRGB(i, 0, 0, 0);
//         }
//     }
//     @Override
//     public void periodic() {
//         dataSetter();
//     }
// }