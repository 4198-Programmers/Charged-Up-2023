package frc.robot.Commands;

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
        double wantedSpeed = speedSupplier.getAsDouble() * speedScalar;
        if (lazySusan.getLocation() <= 80 && wantedSpeed > 0.000001) {
            lazySusan.spinSusan(wantedSpeed);
        } else if (lazySusan.getLocation() >= -80 && wantedSpeed < -0.00001) {
            lazySusan.spinSusan(wantedSpeed);
        } else {
            lazySusan.spinSusan(0);
        }
        // lazySusan.spinSusan(wantedSpeed);
        // System.out.println(lazySusan.getLocation() + "location");

    }

}
