package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class CloseClaw extends CommandBase {
    private final Pneumatics clawSub;

    public CloseClaw(Pneumatics pneumaticsArg) {
        clawSub = pneumaticsArg;
        addRequirements(pneumaticsArg);
    }

    @Override
    public void execute() {
        clawSub.CloseClawPushOut();
        System.out.println("Close");
    }

}
