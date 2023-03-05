package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class ToggleSusan extends CommandBase {
    LazySusanSub lazySusan;
    boolean isFinished;

    public ToggleSusan(LazySusanSub lazySusan) {
        this.lazySusan = lazySusan;
        addRequirements(lazySusan);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        lazySusan.toggleSusan();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
