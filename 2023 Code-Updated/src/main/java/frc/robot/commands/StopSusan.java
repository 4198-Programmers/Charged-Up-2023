package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LazySusanSub;

public class StopSusan extends CommandBase{
    LazySusanSub lazySusan;

    public StopSusan(LazySusanSub susanArg){
        lazySusan = susanArg;
        addRequirements(lazySusan);
    }

    @Override
    public void execute() {
        lazySusan.stopSusan();
    }
    
}
