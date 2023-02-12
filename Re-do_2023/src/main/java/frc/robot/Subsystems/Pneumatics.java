package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    private final Compressor airCompressor;
    private final Solenoid singleSolenoid;
    public Pneumatics(){
        airCompressor = new Compressor(Constants.PNEUMATICS_PORT, PneumaticsModuleType.CTREPCM);
        singleSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PNEUMATICS_SINGLE_CHANNEL);
    }

    //private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 7);

    public void Pressurize() {
        airCompressor.enableDigital();
        System.out.println("Pressure: " + airCompressor.getPressure());
    }

    public void OpenClaw() {
        singleSolenoid.set(true);
        //solenoid.set(Value.kForward);
        System.out.println("Open Claw Made It");
    }

    public void CloseClaw() {
        singleSolenoid.set(false);
        //solenoid.set(Value.kReverse);
        System.out.println("Close Claw Made It");
        //singleSolenoid.close();
    }
    public void toggleChannel(){
        singleSolenoid.set(!singleSolenoid.get());
    }
    public boolean getChannel(){
        return singleSolenoid.get();
    }

    // public void StopClaw() {
    //     solenoid.set(Value.kOff); // essentially an in solenoid
    // }

    // public void setState(Value state) {
    //     gripSolenoid.set(state);
    // }

    // public Value getState() {
    //     return gripSolenoid.get();
    // }
}
