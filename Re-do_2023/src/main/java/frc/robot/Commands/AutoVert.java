package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.VertArm;

public class AutoVert extends CommandBase {
    VertArm vertArm;
    double speed;
    double wantedPos;
    boolean isFinished;

    public AutoVert(VertArm vertArm, double speed, double wantedPos) {
        this.vertArm = vertArm;
        this.speed = speed;
        this.wantedPos = wantedPos;
        addRequirements(vertArm);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        if (vertArm.getLocation() < wantedPos) {// (wantedPos - Constants.AUTO_ENC_OFFSET)
            vertArm.moveArm(0.5);
        } else if (vertArm.getLocation() > (wantedPos + 0.05)) {
            vertArm.moveArm(-0.25);
        } else if (vertArm.getLocation() >= wantedPos
                && vertArm.getLocation() <= (wantedPos + 0.05)) {
            vertArm.moveArm(Constants.VERT_ARM_NO_DROP_SPEED);
            System.out.println("Finished Vert Auto");
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (isFinished);
    }
}
