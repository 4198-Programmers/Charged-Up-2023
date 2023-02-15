package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class SusanHead extends CommandBase{
    LazySusanSub lazySusanSub;
    double head = 0;

    public SusanHead(LazySusanSub lazySusanSub, double head) {
        this.lazySusanSub = lazySusanSub;
        this.head = head;
        addRequirements(lazySusanSub);
    }

    @Override
    public void execute() {
        double differenceInHeading = lazySusanSub.getHeading() - head;
        if(differenceInHeading < -0.5) {
            lazySusanSub.spinSusan(0.1);
        } else if(differenceInHeading > 0.5) {
            lazySusanSub.spinSusan(-0.1);
        }
    }

}
