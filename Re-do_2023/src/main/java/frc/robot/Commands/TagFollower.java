package frc.robot.Commands;


import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.PhotonVision;

public class TagFollower extends CommandBase{
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double wantedYaw;
    private double wantedSkew;
    private double wantedDistance;
    
    public TagFollower(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedYaw, double wantedSkew, double wantedDistance) {
        super();
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedYaw = wantedYaw;
        this.wantedSkew = wantedSkew;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = this.vision.getBestTarget();
        if(target == null) {
            return;
        } else {
            System.out.println(target.getFiducialId());
        }
        //for Chassis Speeds, +vx is foreward, +vy is left, and +omegaRadians is counterclockwise

        //yaw is positive to the right (the tag is to the right relative to the camera)
        double yaw = -target.getYaw();
        double varianceInYaw = this.wantedYaw - yaw;
        //skew is positive when counter clockwise rotation relative to the camera
        double skew = -target.getSkew();
        double varianceInSkew = this.wantedSkew - skew;
        //pitch is positive when it is upwards relative to the camera
        double pitch = -target.getPitch();
        //distance increases as pitch decreases and vice versa
        double distanceToTarget = -Maths.DistanceFromTarget(pitch);
        double varianceInDistance = this.wantedDistance - distanceToTarget;
        double vx = 0;
        double vy = 0;
        double omegaRadians = 0;
        if(varianceInSkew < -0.5) {
            omegaRadians = -0.5;
        } else if (varianceInSkew > 0.5) {
            omegaRadians = 0.5;
        }

        if (varianceInYaw < -0.5) {
            vy = -0.5;
        } else if (varianceInYaw > 0.5) {
            vy = 0.5;
        }

        if(varianceInDistance < -0.5) {
            vx = 0.5;
        } else if (varianceInDistance > 0.5) {
            vx = -0.5;
        } 
        swerveDrive.drive(new ChassisSpeeds(vx, vy, omegaRadians));
        super.execute();
    }

}
