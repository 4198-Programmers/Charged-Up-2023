package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    private final Compressor airCompressor = new Compressor(Constants.PNEUMATICS_PORT, PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.PNEUMATICS_FORWARD_CHANNEL,
            Constants.PNEUMATICS_REVERSE_CHANNEL);

    public void Pressurize() {
        airCompressor.enableDigital();
    }

    public void OpenClawPullIn() {
        gripSolenoid.set(Value.kReverse);
    }

    public void CloseClawPushOut() {
        gripSolenoid.set(Value.kForward);
    }

    public void StopClaw() {
        gripSolenoid.set(Value.kOff); // essentially an in solenoid
    }

    public void setState(Value state) {
        gripSolenoid.set(state);
    }

    public Value getState() {
        return gripSolenoid.get();
    }
}
