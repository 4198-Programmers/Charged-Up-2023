package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class ZeroSusan extends CommandBase{
    LazySusanSub susan;

    public ZeroSusan(LazySusanSub susanSub){
        this.susan = susanSub;
        addRequirements(susanSub);
    }

    @Override
    public void execute() {
        susan.stopSusan();
        susan.zeroPosition();
    }

    @Override
    public boolean isFinished() {
        return susan.getLocation() == 0;
    }
    
}
