package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class ShoulderMove extends CommandBase{
    double speed;
    Shoulder shoulder;
    public ShoulderMove(double speed, Shoulder shoulder){
        this.speed = speed;
        this.shoulder = shoulder;
        addRequirements(shoulder);
    }
    @Override
    public void execute() {
        shoulder.setSpeed(speed);
    }
}
