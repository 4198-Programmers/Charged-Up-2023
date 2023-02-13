package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class ToggleChannels extends CommandBase{
    Pneumatics pneumatics;
    boolean channel;
    public ToggleChannels(Pneumatics pneumatics, boolean channel){
        this.pneumatics = pneumatics;
        this.channel = channel;
        addRequirements(pneumatics);
    }
    @Override
    public void execute() {
       pneumatics.togglePneumatics(channel);
    }
}
