package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class HandMove extends CommandBase{
    double speed;
    String value;
    Claw claw;

    public HandMove(String value, Claw claw){
        this.value = value;
        this.claw = claw;
        addRequirements(claw);
    }

@Override
public void execute() {
    claw.useClaw(value);
}
}
