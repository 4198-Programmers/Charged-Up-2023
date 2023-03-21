package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ReachArmSub;

public class AutoReach extends CommandBase{
    Timer timer;
    ReachArmSub reachArmSub;
    double wantedTime;
    double speed;
    public AutoReach(ReachArmSub reachArmSub, double wantedTime, double speed){
        this.reachArmSub = reachArmSub;
        this.wantedTime = wantedTime;
        this.speed = speed;
        addRequirements(reachArmSub);
    }
    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
    }
    @Override
    public void execute() {
        reachArmSub.moveReach(speed);
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= wantedTime;
    }
}
