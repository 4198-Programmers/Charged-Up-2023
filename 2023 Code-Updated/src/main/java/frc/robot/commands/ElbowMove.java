package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;

public class ElbowMove extends CommandBase{
    double speed;
    Elbow elbow;
    public ElbowMove(double speed, Elbow elbow){
        this.speed = speed;
        this.elbow = elbow;
        addRequirements(elbow);
    }
    @Override
    public void execute() {
        elbow.setSpeed(speed);
    }
}
