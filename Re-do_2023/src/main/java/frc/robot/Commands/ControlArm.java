package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.VertArm;

public class ControlArm extends CommandBase {
    private final VertArm upArmSub;
    private final DoubleSupplier speedSupplier;
    private double speedScalar;
    private double percentSpeed;

    public ControlArm(VertArm upArmArg, DoubleSupplier supplier, double percentSpeed) {
        upArmSub = upArmArg;
        speedSupplier = supplier;
        this.percentSpeed = percentSpeed;
        addRequirements(upArmArg);
    }

    @Override
    public void initialize() {
        speedScalar = percentSpeed / 100;
    }

    @Override
    public void execute() {
        System.out.println(upArmSub.getLocation() + "Up Arm");
        upArmSub.moveArm(speedSupplier.getAsDouble() * speedScalar);
    }

}
