package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class HandMove extends CommandBase{
    double speed;
    Hand hand;

    public HandMove(double speed, Hand hand){
        this.speed = speed;
        this.hand = hand;
        addRequirements(hand);
    }

    @Override
    public void initialize() {
        hand.setSpeed(speed);
    }
}
