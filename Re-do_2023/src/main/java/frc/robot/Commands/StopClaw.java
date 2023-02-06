package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class StopClaw extends CommandBase {
    private final Pneumatics pneumaticsSub;

    public StopClaw(Pneumatics pneumaticsArg) {
        pneumaticsSub = pneumaticsArg;
        addRequirements(pneumaticsArg);
    }

    @Override
    public void execute() {
        pneumaticsSub.StopClaw();
    }

}
