package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    // set up the camera object
    PhotonCamera camera = new PhotonCamera("Limelight");
    // essentially collects the latest data from the camera
    PhotonPipelineResult result = camera.getLatestResult();
    
    
    
    double skew;
    double yaw;
    double pitch;
    double fiducialId;

    public PhotonTrackedTarget getBestTarget() {
        PhotonTrackedTarget target = null; 
        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.println(target.toString());
        }
        return target;
    }
}
