package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

public class ControlVertStraightDown extends CommandBase {
    private final VertArm vertArm;
    private final DoubleSupplier speedSupplier;
    private double speedScalar;
    private double percentSpeed;
    private final ReachArmSub reachArm;

    public ControlVertStraightDown(VertArm upArmArg, ReachArmSub reachArm, DoubleSupplier supplier,
            double percentSpeed) {
        this.vertArm = upArmArg;
        this.reachArm = reachArm;
        this.speedSupplier = supplier;
        this.percentSpeed = percentSpeed;
        addRequirements(upArmArg, reachArm);
    }

    @Override
    public void initialize() {
        speedScalar = percentSpeed / 100;
    }

    @Override
    public void execute() {
        double wantedSpeed = speedSupplier.getAsDouble() * speedScalar;
        double reachWantedPos = 15907.1 * (Math.pow(0.5007, vertArm.getLocation()));

        if (reachArm.getPosition() < reachWantedPos - Constants.REACH_ENCODER_TOLERANCE) {
            reachArm.moveReach(0.5);
        } 
        // else if (reachArm.getPosition() > reachWantedPos + Constants.REACH_ENCODER_TOLERANCE) {
        //     reachArm.moveReach(-wantedSpeed);
        // } 
        else {
            reachArm.moveReach(0);
        }

        System.out.println(reachWantedPos + "Equation Pos");
        System.out.println(reachArm.getPosition() + "Current Pos");

        if (vertArm.getLocation() <= 7.925 && wantedSpeed > 0.05) {
            vertArm.moveArm(wantedSpeed);
        } else if (vertArm.getLocation() > 0 && wantedSpeed < -0.05) {
            vertArm.moveArm(wantedSpeed);
        } else {
            vertArm.moveArm(.25 * wantedSpeed);
            reachArm.moveReach(0);
        }
    }

}
