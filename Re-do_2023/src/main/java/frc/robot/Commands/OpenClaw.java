package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;


public class OpenClaw extends CommandBase {
    private final Pneumatics pneumaticsSub;

    public OpenClaw(Pneumatics clawArg) {
        pneumaticsSub = clawArg;
        addRequirements(clawArg);
    }

    @Override
    public void execute() {
        pneumaticsSub.OpenClaw();
        System.out.println("Open");
    }

}
