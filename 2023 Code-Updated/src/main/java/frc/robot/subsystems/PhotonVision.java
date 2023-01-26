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
    
    PhotonTrackedTarget target;
    
    double skew;
    double yaw;
    double pitch;
    double fiducialId;

    public PhotonTrackedTarget getBestTarget() {
        this.target = null; 
        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.println(target.toString());
        }
        return target;
    }

    public double getYaw() {
        this.yaw = this.target.getYaw();
        return this.yaw;
    }

    public double getPitch() {
        this.pitch = this.target.getPitch();
        return this.pitch;
    }

    public double getSkew() {
        this.skew = this.target.getSkew();
        return this.skew;
    }
}
