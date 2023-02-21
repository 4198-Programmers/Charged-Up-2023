package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class SusanHead extends CommandBase{
    LazySusanSub lazySusanSub;
    double head;

    public SusanHead(LazySusanSub lazySusanSub, double head) {
        this.lazySusanSub = lazySusanSub;
        this.head = head;
        addRequirements(lazySusanSub);
    }

    @Override
    public void execute() {
        lazySusanSub.spinSusanWithAngles(0.25, head);
    }

}
