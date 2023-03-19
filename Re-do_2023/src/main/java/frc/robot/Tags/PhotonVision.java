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
    double skew;
    double yaw;
    double pitch;
    double fiducialId;

    public PhotonTrackedTarget getTarget() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.println("Target Data: " + target.getFiducialId());
        } else {
            target = null;
        }
        return target;
    }

    public double getYaw() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            yaw = target.getYaw();
        } else {
            yaw = 0;
        }
        return yaw;
    }

    public double getPitch() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            pitch = target.getPitch();
        } else {
            pitch = 0;
        }
        return pitch;
    }

    public double getSkew() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            skew = target.getSkew();
        } else {
            skew = 0;
        }
        return this.skew;
    }

    @Override
    public void periodic() {
        getTarget();
        SmartDashboard.putNumber("Pitch: ", getPitch());
        SmartDashboard.putNumber("Yaw: ", getYaw());
        SmartDashboard.putNumber("Skew: ", getSkew());
    }
}
