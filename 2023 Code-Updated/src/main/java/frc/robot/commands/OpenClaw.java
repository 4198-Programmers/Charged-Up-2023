package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class OpenClaw extends CommandBase{
    Pneumatics claw;

    public OpenClaw(Pneumatics clawArg){
        claw = clawArg;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        // claw.OpenClawInSolenoid();
    }
    
}
