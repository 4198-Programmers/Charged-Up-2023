package frc.robot.Commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class SusanMode extends CommandBase{
    LazySusanSub lazySusanSub;
    IdleMode mode;
    
    public SusanMode(LazySusanSub lazySusanSub, IdleMode mode){
        this.lazySusanSub = lazySusanSub;
        this.mode = mode;
        addRequirements(lazySusanSub);
    }
    @Override
    public void execute() {
        lazySusanSub.mode(mode);
    }
}
