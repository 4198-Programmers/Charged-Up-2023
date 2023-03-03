package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Grabber;

public class ControlGrabber extends CommandBase{
    Grabber grabber;
    double speed;
    public ControlGrabber(Grabber grabber, double speed){
        this.grabber = grabber;
        this.speed = speed;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        grabber.setSpeed(speed);
    }
}
