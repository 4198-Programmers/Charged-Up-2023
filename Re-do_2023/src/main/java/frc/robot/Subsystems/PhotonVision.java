package frc.robot.Subsystems;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
    // set up the camera object
    PhotonCamera camera = new PhotonCamera("Limelight");
    // essentially collects the latest data from the camera
    PhotonPipelineResult result = camera.getLatestResult();
    
    PhotonTrackedTarget target;
    AprilTagFieldLayout layout;
    Transform3d robotToCamera;

    PhotonPoseEstimator estimator;
    double skew;
    double yaw;
    double pitch;
    double fiducialId;

    public PhotonVision() {
        super();
        try {
            this.layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);            
        } catch (IOException e) {
            // TODO: handle exception
        }
        robotToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(Constants.TRANSLATION_3D_X), Units.inchesToMeters(Constants.TRANSLATION_3D_Y), Units.inchesToMeters(Constants.TRANSLATION_3D_Z)),
        new Rotation3d(Constants.ROTATION_3D_ROLL, Constants.ROTATION_3D_PITCH, Constants.ROTATION_3D_YAW));
        estimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCamera);
        
    }

    public PhotonTrackedTarget getBestTarget() {
        this.target = null; 
        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.println(target.toString());
        }
        return target;
    }


    public Optional<EstimatedRobotPose> update() {
        return estimator.update();
    }

    @Override
    public void periodic() {
        var target = getBestTarget();
        update();
        if(target == null) {
            SmartDashboard.putString("Target: ", "No Target");
            SmartDashboard.putNumber("Pitch: ", 0);
            SmartDashboard.putNumber("Yaw: ", 0);
            SmartDashboard.putNumber("Skew: ", 0);
            return;
        } 
        SmartDashboard.putString("Target: ", "ID: " + target.getFiducialId());
        SmartDashboard.putNumber("Pitch: ", target.getPitch());
        SmartDashboard.putNumber("Yaw: ", target.getYaw());
        SmartDashboard.putNumber("Skew: ", target.getSkew());  
        
        
    }
}
