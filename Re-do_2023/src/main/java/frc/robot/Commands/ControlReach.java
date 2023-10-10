package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ReachArmSub;

public class ControlReach extends CommandBase {
    private final ReachArmSub reach;
    private final DoubleSupplier speedSupplier;
    private double percentSpeed;
    private double speedScalar;

    public ControlReach(ReachArmSub reachArg, DoubleSupplier supplier, double percentSpeed) {
        reach = reachArg;
        speedSupplier = supplier;
        this.percentSpeed = percentSpeed;
        addRequirements(reachArg);
    }

    @Override
    public void initialize() {
        speedScalar = Math.abs(percentSpeed / 100);
    }

    @Override
    public void execute() {
        reach.moveReach(speedSupplier.getAsDouble() * speedScalar);
        // System.out.println("Reach Arm Speed: " + reach.getSpeed());
        System.out.println("Reach Arm Position: " + reach.getPosition());


    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
