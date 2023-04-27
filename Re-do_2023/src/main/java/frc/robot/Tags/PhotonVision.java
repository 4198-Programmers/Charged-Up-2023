package frc.robot.Tags;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVision extends SubsystemBase {
    // set up the camera object
    PhotonCamera camera = new PhotonCamera("photonvision");
    // essentially collects the latest data from the camera
    PhotonPipelineResult result;

    PhotonTrackedTarget target;
    double fiducialId;

    public PhotonTrackedTarget getTarget() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
        } else {
            target = null;
        }


        return target;
    }


}
