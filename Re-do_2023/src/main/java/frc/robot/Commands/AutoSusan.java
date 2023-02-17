package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.LazySusanSub;

public class AutoSusan extends CommandBase {
    LazySusanSub lazySusanSub;
    double speed;
    double wantedDegrees;

    public AutoSusan(LazySusanSub lazySusanSub, double speed, double wantedDegrees) {
        this.lazySusanSub = lazySusanSub;
        this.speed = speed;
        this.wantedDegrees = wantedDegrees;
        addRequirements(lazySusanSub);
    }

    @Override
    public void execute() {
        lazySusanSub.spinSusanWithAngles(speed, wantedDegrees);
    }

    @Override
    public boolean isFinished() {
        return -Constants.ANGLE_OFFSET <= (lazySusanSub.getRotation() - wantedDegrees) &&
                (lazySusanSub.getRotation() - wantedDegrees) <= Constants.ANGLE_OFFSET;
    }
}
