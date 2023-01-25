package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class DistanceTag extends CommandBase {
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private boolean isfinished = false;
    public DistanceTag(PhotonVision photonVision, DriveTrain swerveDriveTrain) {
        super();
        this.vision = photonVision;
        this.swerveDrive = swerveDriveTrain;
        addRequirements(photonVision, swerveDriveTrain);
    }
    
    @Override
    public void execute() {
        PhotonTrackedTarget target = this.vision.getBestTarget();
        if(target == null) {
            isfinished = true;
            return;
        }
        double distanceToTarget = Maths.DistanceFromTarget(target.getPitch());
        double varianceInDistance = Constants.WANTED_DISTANCE - distanceToTarget;
        if(varianceInDistance < -0.5) {
            swerveDrive.Move(0, 0.5);
        } else if(varianceInDistance > 0.5) {
            swerveDrive.Move(180, 0.5);
        } else {
            swerveDrive.Move(0, 0);
            this.isfinished = true;
        }
    
    }
    @Override
    public boolean isFinished() {
        return this.isfinished;
    }
}
