package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class ToggleChannels extends CommandBase{
    Pneumatics pneumatics;
    boolean channel;
    public ToggleChannels(Pneumatics pneumatics){
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }
    @Override
    public void initialize() {
       channel = pneumatics.getChannel();
    }
    @Override
    public void execute() {
        pneumatics.togglePneumatics(!channel);
    }
}
