package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class CloseClaw extends CommandBase{
    Pneumatics claw;

    public CloseClaw(Pneumatics clawArg){
        claw = clawArg;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        // claw.CloseClawOutSolenoid();
    }
    
}
