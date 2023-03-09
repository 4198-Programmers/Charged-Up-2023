package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ReachArmSub;

public class AutoReach extends CommandBase {
    ReachArmSub reachArm;
    double speed;
    boolean isFinished;
    long startTime;
    long timeRun;

    public AutoReach(ReachArmSub reachArm, double speed, long timeRunMill) {
        this.reachArm = reachArm;
        this.speed = speed;
        this.timeRun = timeRunMill;
        addRequirements(reachArm);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        isFinished = false;
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime < timeRun) {// (wantedPos - Constants.AUTO_ENC_OFFSET)
            reachArm.moveReach(speed);
        } else {
            reachArm.moveReach(0);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (isFinished);
    }
}
