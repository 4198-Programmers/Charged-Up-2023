package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.VertArm;

public class ControlArm extends CommandBase {
    private final VertArm upArmSub;
    private final DoubleSupplier speedSupplier;
    private final double speedScalar;

    public ControlArm(VertArm upArmArg, DoubleSupplier supplier, int percentSpeed) {
        upArmSub = upArmArg;
        speedSupplier = supplier;
        speedScalar = percentSpeed / 100;
        addRequirements(upArmArg);
    }

    @Override
    public void execute() {
        upArmSub.moveArm(speedSupplier.getAsDouble() * speedScalar);
        System.out.println("VertArm " + upArmSub.getLocation());

    }

}
