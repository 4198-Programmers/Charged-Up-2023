package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    // CANSparkMax handMotor = new CANSparkMax(Constants.HAND_MOTOR, MotorType.kBrushless);
    // RelativeEncoder handEncoder = handMotor.getEncoder();

    // public void setSpeed(double speed){
    //     handMotor.set(speed);
    // }
    // public double getLocation(){
    //     return handEncoder.getPosition();
    // }
    // public void setSpeedWithLimits(double speed){
    //     double expectedSpeed = speed;
    //     if(speed > 0 && getLocation() >= Constants.HAND_UPPER_LIMIT){
    //         expectedSpeed = 0;
    //     }
    //     else if(speed < 0 && getLocation() <= Constants.HAND_LOWER_LIMIT){
    //         expectedSpeed = 0;
    //     }
    //     handMotor.set(expectedSpeed);
    // }
    int forwardChannel;
    int reverseChannel;
    int compressorModule;
    Compressor airCompressor = new Compressor(compressorModule, PneumaticsModuleType.CTREPCM);
    DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    public void Pressurize(){
        airCompressor.enableDigital();
    }
    public void useClaw(String value){
        clawSolenoid.set(Value.valueOf(value));
    }


}
