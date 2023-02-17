package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.VertArm;

public class AutoVert extends CommandBase{
    VertArm vertArm;
    double speed;
    double wantedDistance;

    public AutoVert(VertArm vertArm, double speed, double wantedDistance){
        this.vertArm = vertArm;
        this.speed = speed;
        this.wantedDistance = wantedDistance;
    }
    @Override
    public void execute() {
        vertArm.autoVert(speed, wantedDistance);
    }
    @Override
    public boolean isFinished() {
        return wantedDistance + Constants.VERT_OFFSET >= vertArm.getLocation() && vertArm.getLocation() >= wantedDistance - Constants.VERT_OFFSET;
    }
}
