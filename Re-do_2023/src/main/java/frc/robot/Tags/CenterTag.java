package frc.robot.Tags;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveTrain;

public class CenterTag extends CommandBase {
    // left and right centering
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    double wantedDistance;

    private boolean isfinished = false;

    public CenterTag(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedDistance) {
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedDistance = wantedDistance;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = vision.getTarget();
        if (target == null) {
            isfinished = true;
            return;
        }
        double yaw = target.getYaw();
        double distanceToCenter = wantedDistance - yaw;
        if (distanceToCenter < -Constants.PHOTON_TOLERANCE_VALUE) {
            swerveDrive.drive(new ChassisSpeeds(0, -0.5, 0));
            ;
        } else if (distanceToCenter > Constants.PHOTON_TOLERANCE_VALUE) {
            swerveDrive.drive(new ChassisSpeeds(0, 0.5, 0));
        } else {
            isfinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isfinished;
    }
}
