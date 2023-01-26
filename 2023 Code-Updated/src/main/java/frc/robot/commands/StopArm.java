package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VertArm;

public class StopArm extends CommandBase{
    VertArm vertArm;

    public StopArm(VertArm vertArmArg) {
        vertArm = vertArmArg;
        addRequirements(vertArm);
    }

    @Override
    public void execute() {
        vertArm.stopArm();
    }
    
}
