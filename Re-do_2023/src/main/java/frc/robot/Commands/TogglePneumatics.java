package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class TogglePneumatics extends CommandBase {
    Pneumatics pneumatics;
    boolean channel;

    public TogglePneumatics(Pneumatics pneumatics, boolean channel) { // False is closed [2-24]
        this.pneumatics = pneumatics;
        this.channel = channel;
        addRequirements(pneumatics);
    }

    @Override
    public void execute() {
        pneumatics.togglePneumatics(channel);
    }

    @Override
    public boolean isFinished() {
        return pneumatics.getChannel() == channel;
    }
}
