package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    private final Compressor airCompressor;
    private final Solenoid singleSolenoid;

    public Pneumatics() {
        airCompressor = new Compressor(Constants.PNEUMATICS_PORT, PneumaticsModuleType.CTREPCM);
        singleSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PNEUMATICS_SINGLE_CHANNEL);
    }

    // private final DoubleSolenoid solenoid = new
    // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 7);

    public void Pressurize() {
        // airCompressor.enableDigital();
        airCompressor.disable();
        System.out.println("Pressure: " + airCompressor.getPressure());
    }

    public boolean getChannel() {
        return singleSolenoid.get();
    }

    public void togglePneumatics(boolean solenoidOn) {
        singleSolenoid.set(solenoidOn);
    }
}
