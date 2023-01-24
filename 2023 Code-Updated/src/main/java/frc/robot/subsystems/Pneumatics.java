package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase{
    Compressor airCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    public void Pressurize(){
        airCompressor.enableDigital();
    }

    public void OpenClawInSolenoid(){
        clawSolenoid.set(Value.kReverse);
    }

    public void CloseClawOutSolenoid(){
        clawSolenoid.set(Value.kForward);
    }

    public void StopClawSolenoid(){
        clawSolenoid.set(Value.kOff);
    }
    
}
