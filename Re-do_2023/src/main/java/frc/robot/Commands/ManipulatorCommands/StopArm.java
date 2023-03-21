package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.VertArm;

public class StopArm extends CommandBase {
    private final VertArm vertArmSub;

    public StopArm(VertArm vertArmArg) {
        vertArmSub = vertArmArg;
        addRequirements(vertArmArg);
    }

    @Override
    public void execute() {
        vertArmSub.stopArm();
    }

}
