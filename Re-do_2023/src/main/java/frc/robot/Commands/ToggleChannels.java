package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class ToggleChannels extends CommandBase{
    Pneumatics pneumatics;
    public ToggleChannels(Pneumatics pneumatics){
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }
    @Override
    public void execute() {
        pneumatics.toggleChannel();
        System.out.println("Toggling");
        
        System.out.println("After Toggle: " + pneumatics.getChannel());
    }
}
