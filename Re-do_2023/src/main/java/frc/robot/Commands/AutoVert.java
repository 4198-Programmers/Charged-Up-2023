package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.VertArm;

public class AutoVert extends CommandBase {
    VertArm vertArm;
    double speed;
    double wantedPos;

    public AutoVert(VertArm vertArm, double speed, double wantedPos) {
        this.vertArm = vertArm;
        this.speed = speed;
        this.wantedPos = wantedPos;
    }

    @Override
    public void execute() {
        System.out.println(vertArm.getLocation() + "Vert Auto");
        if (vertArm.getLocation() < wantedPos - Constants.AUTO_ENC_OFFSET) {
            vertArm.moveArm(speed);
        } else if (vertArm.getLocation() > wantedPos + Constants.AUTO_ENC_OFFSET) {
            vertArm.moveArm(-speed);
        } else {
            vertArm.moveArm(0);
        }
    }

    @Override
    public boolean isFinished() {
        return (vertArm.getLocation() >= wantedPos - Constants.AUTO_ENC_OFFSET
                && vertArm.getLocation() <= wantedPos + Constants.AUTO_ENC_OFFSET);
    }
}
