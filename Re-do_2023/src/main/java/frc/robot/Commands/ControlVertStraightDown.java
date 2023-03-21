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
    private double initialReachPos;
    private double wantedSpeed;
    private double reachWantedPos;
    private double r = 3805.44;
    private double b = -0.721424;
    private double a = 0.485838;
    private double vertArmPos;
    private double reachArmPos;

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
        initialReachPos = reachArm.getPosition();
    }

    @Override
    public void execute() {
        wantedSpeed = speedSupplier.getAsDouble() * speedScalar;
        vertArmPos = vertArm.getLocation();
        reachArmPos = reachArm.getPosition();
        // double reachWantedPos = 15907.1 * (Math.pow(0.5007, vertArm.getLocation()));
        // // old
        reachWantedPos = (r + initialReachPos)
                * (1 - (b * Math.cos(a * vertArmPos)));

        if (vertArmPos <= 8.25 && wantedSpeed > 0.05) {
            vertArm.moveArm(wantedSpeed);
        } else if (vertArmPos > 0 && wantedSpeed < -0.05) {
            vertArm.moveArm(wantedSpeed);
        } else {
            vertArm.moveArm(.25 * wantedSpeed);
            reachArm.moveReach(0);
        }

        if (reachArmPos < reachWantedPos - 50) {
            reachArm.moveReach(0.75);
        } else if (reachArmPos > reachWantedPos + 50) { // encoder tolerance for less shake
            reachArm.moveReach(-1);
        } else {
            reachArm.moveReach(0);
        }

        // System.out.println(reachWantedPos + "Equation Pos");
        // System.out.println(reachArm.getPosition() + "Current Pos");

    }

}
