package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.LazySusanSub;

public class AutoSusan extends CommandBase {
    LazySusanSub lazySusanSub;
    double speed;
    double wantedPos;

    public AutoSusan(LazySusanSub lazySusanSub, double speed, double wantedPos) {
        this.lazySusanSub = lazySusanSub;
        this.speed = speed;
        this.wantedPos = wantedPos;
        addRequirements(lazySusanSub);
    }

    @Override
    public void execute() {
        if (lazySusanSub.getLocation() < wantedPos - Constants.AUTO_ENC_OFFSET) {
            lazySusanSub.spinSusan(-speed);
        } else if (lazySusanSub.getLocation() > wantedPos + Constants.AUTO_ENC_OFFSET) {
            lazySusanSub.spinSusan(speed);
        } else {
            lazySusanSub.spinSusan(0);
        }
    }

    @Override
    public boolean isFinished() {
        return (lazySusanSub.getLocation() >= wantedPos - Constants.AUTO_ENC_OFFSET
                && lazySusanSub.getLocation() <= wantedPos + Constants.AUTO_ENC_OFFSET);
    }
}
