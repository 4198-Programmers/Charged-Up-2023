package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.VertArm;

public class ZeroVert extends CommandBase{
    VertArm vertArm;

    public ZeroVert(VertArm vertArmArg){
        this.vertArm = vertArmArg;
        addRequirements(vertArmArg);
    }

    @Override
    public void execute() {
        vertArm.stopArm();
        vertArm.ZeroArm();
    }
    
}
