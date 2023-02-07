package frc.robot.commands;


import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class CenterTag extends CommandBase{
    private PhotonVision vision;
    private DriveTrain swerveDrive;

    private boolean isfinished = false;
    
    public CenterTag(PhotonVision visionSub, DriveTrain swerveDriveSub) {
        super();
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = this.vision.getBestTarget();
        if(target == null) {
            this.isfinished = true;
            return;
        }
        double yaw = target.getYaw();
        double varianceInYaw = Constants.WANTED_YAW - yaw;
        if(varianceInYaw < -0.5) {
            swerveDrive.Move(-90, 0.5);
        } else if (varianceInYaw > 0.5) {
            swerveDrive.Move(90, 0.5);
        } else {
            this.isfinished = true;
        }
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return isfinished;
    }
}
