package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.VertArm;

public class ControlArm extends CommandBase {
    private final VertArm vertArm;
    private final DoubleSupplier speedSupplier;
    private double speedScalar;
    private double percentSpeed;

    public ControlArm(VertArm upArmArg, DoubleSupplier supplier, double percentSpeed) {
        vertArm = upArmArg;
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
        double wantedSpeed = speedSupplier.getAsDouble() * speedScalar;
        if (vertArm.getLocation() <= 8.25 && wantedSpeed > 0.05) {
            vertArm.moveArm(wantedSpeed);
        } else if (vertArm.getLocation() > 0 && wantedSpeed < -0.05) {
            vertArm.moveArm(wantedSpeed);
        } else {
            vertArm.moveArm(.25 * wantedSpeed);
        }
        // System.out.println("Arm Speed: " + vertArm.getSpeed());
        // System.out.println("Vert Arm Position: " + vertArm.getPosition());
    }

}
