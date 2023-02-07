package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RetractableArm;

public class RetractableArmMove extends CommandBase{
    double speed;
    RetractableArm retractableArm;

    public RetractableArmMove(double speed, RetractableArm retractableArm){
        this.speed = speed;
        this.retractableArm = retractableArm;
        addRequirements(retractableArm);
    }

    @Override
    public void execute() {
        retractableArm.setSpeed(speed);
    }
}
