package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.LazySusanSub;

public class HoldSusan extends CommandBase {
    LazySusanSub lazySusanSub;
    double speed;
    double wantedPos;

    public HoldSusan(LazySusanSub lazySusanSub, double speed, double wantedPos) {
        this.lazySusanSub = lazySusanSub;
        this.speed = speed;
        this.wantedPos = wantedPos;
        addRequirements(lazySusanSub);
    }

    @Override
    public void execute() {
        // if (lazySusanSub.getLocation() < wantedPos - Constants.AUTO_ENC_OFFSET && lazySusanSub.getLocation() <= 80) {
        //     lazySusanSub.spinSusan(speed);
        // } else if (lazySusanSub.getLocation() > wantedPos + Constants.AUTO_ENC_OFFSET
        //         && lazySusanSub.getLocation() >= -80) {
        //     lazySusanSub.spinSusan(-speed);
        // } else {
        //     lazySusanSub.spinSusan(0);
        // }
        if (lazySusanSub.getLocation() < wantedPos - 0.05 && lazySusanSub.getLocation() <= 80) {
            lazySusanSub.susanEquationSpin(wantedPos, speed);
        } else if (lazySusanSub.getLocation() > wantedPos + 0.05
                && lazySusanSub.getLocation() >= -80) {
                    lazySusanSub.susanEquationSpin(wantedPos, -speed);
                } else {
            lazySusanSub.spinSusan(0);
        }
    }

    @Override
    public boolean isFinished() {
        return lazySusanSub.susanDisable || false;
    }
}
