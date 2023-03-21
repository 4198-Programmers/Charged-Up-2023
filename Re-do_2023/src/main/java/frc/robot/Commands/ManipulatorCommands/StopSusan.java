package frc.robot.Commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class StopSusan extends CommandBase {
    private final LazySusanSub lazySusan;

    public StopSusan(LazySusanSub susanArg) {
        lazySusan = susanArg;
        addRequirements(susanArg);
    }

    @Override
    public void execute() {
        lazySusan.stopSusan();
    }

}
