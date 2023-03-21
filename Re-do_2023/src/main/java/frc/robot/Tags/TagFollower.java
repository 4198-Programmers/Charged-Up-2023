package frc.robot.Tags;


import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Tags.PhotonVision;

public class TagFollower extends CommandBase{
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double wantedYaw;
    private double wantedSkew;
    private double wantedDistance;
    PhotonTrackedTarget target;
    private boolean xFinished;
    private boolean yFinished;
    private boolean zFinished;
    
    public TagFollower(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedYaw, double wantedSkew, double wantedDistance) {
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedYaw = wantedYaw;
        this.wantedSkew = wantedSkew;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void initialize() {
        xFinished = false;
        yFinished = false;
        zFinished = false;
    }

    @Override
    public void execute() {
        target = vision.getTarget();

        if(target == null) {
            xFinished = true;
            yFinished = true;
            zFinished = true;
        }
        //for Chassis Speeds, +vx is foreward, +vy is left, and +omegaRadians is counterclockwise

        //yaw is positive to the right (the tag is to the right relative to the camera)
        double yaw = -target.getYaw();
        double varianceInYaw = wantedYaw - yaw;
        //skew is positive when counter clockwise rotation relative to the camera
        double skew = -target.getSkew();
        double varianceInSkew = wantedSkew - skew;
        //pitch is positive when it is upwards relative to the camera
        double pitch = -target.getPitch();
        //distance increases as pitch decreases and vice versa
        double distanceToTarget = -Maths.DistanceFromTarget(pitch);
        double varianceInDistance = wantedDistance - distanceToTarget;
        double vx = 0;
        double vy = 0;
        double omegaRadians = 0;

        if(varianceInSkew < -0.5) {
            omegaRadians = -0.5;
        } else if (varianceInSkew > 0.5) {
            omegaRadians = 0.5;
        } else{
            omegaRadians = 0;
            zFinished = true;
        }

        if (varianceInYaw < -0.5) {
            vy = -0.5;
        } else if (varianceInYaw > 0.5) {
            vy = 0.5;
        } else{
            vy = 0;
            yFinished = true;
        }

        if(varianceInDistance < -0.5) {
            vx = 0.5;
        } else if (varianceInDistance > 0.5) {
            vx = -0.5;
        } else{
            vx = 0;
            xFinished = true;
        }

        swerveDrive.drive(new ChassisSpeeds(vx, vy, omegaRadians));
    }

}
