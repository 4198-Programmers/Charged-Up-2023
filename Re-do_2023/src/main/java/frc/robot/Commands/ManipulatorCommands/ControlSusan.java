package frc.robot.Commands.ManipulatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.LazySusanSub;

public class ControlSusan extends CommandBase {
    private final LazySusanSub lazySusan;
    private final DoubleSupplier speedSupplier;
    private double speedScalar;
    private final double percentSpeed;

    public ControlSusan(LazySusanSub susanArg, DoubleSupplier supplier, double percentSpeed) {
        this.percentSpeed = percentSpeed;
        lazySusan = susanArg;
        speedSupplier = supplier;
        addRequirements(lazySusan);
    }

    @Override
    public void initialize() {
        speedScalar = Math.abs(percentSpeed / 100); // How can math have anti-lock braking?
    }

    @Override
    public void execute() {
        lazySusan.spinSusan(speedSupplier.getAsDouble() * speedScalar);
    }

}
