package frc.robot.Tags;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;

public class DistanceTag extends CommandBase {
    // forward and backward centering
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    double wantedDistance;

    private boolean isfinished = false;

    public DistanceTag(PhotonVision photonVision, DriveTrain swerveDriveTrain, double wantedDistance) {
        this.vision = photonVision;
        this.swerveDrive = swerveDriveTrain;
        this.wantedDistance = wantedDistance;
        addRequirements(photonVision, swerveDriveTrain);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = vision.getTarget();
        if (target == null) {
            isfinished = true;
            return;
        }
        double distanceFromTarget = Maths.DistanceFromTarget(vision.getPitch());
        double distanceToTarget = wantedDistance - distanceFromTarget;
        if (distanceToTarget < -Constants.DISTANCE_TOLERANCE_VALUE) {
            swerveDrive.drive(new ChassisSpeeds(-0.5, 0, 0));
            ;
        } else if (distanceToTarget > Constants.DISTANCE_TOLERANCE_VALUE) {
            swerveDrive.drive(new ChassisSpeeds(0.5, 0, 0));
            ;
        } else {
            isfinished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return isfinished;
    }
}
