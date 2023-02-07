package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class ControlSusan extends CommandBase {
    private final LazySusanSub lazySusan;
    private final DoubleSupplier speedSupplier;
    private final double speedScalar;

    public ControlSusan(LazySusanSub susanArg, DoubleSupplier supplier, int percentSpeed) {
        lazySusan = susanArg;
        speedSupplier = supplier;
        speedScalar = percentSpeed / 100;
        addRequirements(lazySusan);
    }

    @Override
    public void execute() {
        lazySusan.spinSusan(speedSupplier.getAsDouble() * speedScalar);
    }

}