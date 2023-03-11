package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ReachArmSub;

public class AutoReach extends CommandBase {
    ReachArmSub reachArm;
    double speed;
    boolean isFinished;
    double wantedPos;
    // long startTime;
    // long timeRun;

    public AutoReach(ReachArmSub reachArm, double speed, double wantedPos) {
        this.reachArm = reachArm;
        this.speed = speed;
        this.wantedPos = wantedPos;
        addRequirements(reachArm);
    }

    @Override
    public void initialize() {
        // startTime = System.currentTimeMillis();
        isFinished = false;
    }

    @Override
    public void execute() {
        // if (System.currentTimeMillis() - startTime < timeRun) {// (wantedPos -
        // Constants.AUTO_ENC_OFFSET)
        // reachArm.moveReach(speed);
        // } else {
        // reachArm.moveReach(0);
        // isFinished = true;
        // }
        if (reachArm.getPosition() < wantedPos + Constants.REACH_ENCODER_TOLERANCE
                && reachArm.getPosition() > wantedPos - Constants.REACH_ENCODER_TOLERANCE) {
                reachArm.moveReach(0);
            isFinished = true;
        } else if (reachArm.getPosition() < wantedPos - Constants.REACH_ENCODER_TOLERANCE) {
            reachArm.moveReach(speed);
        } else if (reachArm.getPosition() > wantedPos+Constants.REACH_ENCODER_TOLERANCE) {
            reachArm.moveReach(-speed);
        }

        System.out.println(reachArm.getPosition());
    }

    @Override
    public boolean isFinished() {
        return (isFinished);
    }
}
