package frc.robot.Tags;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;

public class CenterSusanPhoton extends CommandBase {
    private PhotonVision vision;
    private DriveTrain driveTrain;
    private LazySusanSub lazySusan;
    private double wantedYaw;
    private double wantedSkew;
    private double wantedDistance;

    public CenterSusanPhoton(PhotonVision visionSub, DriveTrain driveTrainSub, LazySusanSub lazySusanSub,
            double wantedYaw, double wantedSkew, double wantedDistance) {
        this.vision = visionSub;
        this.driveTrain = driveTrainSub;
        this.lazySusan = lazySusanSub;
        this.wantedYaw = wantedYaw;
        this.wantedSkew = wantedSkew;
        addRequirements(visionSub, driveTrainSub);
    }

    @Override
    public void execute() {
        System.out.println(vision.getTarget());
        PhotonTrackedTarget target = vision.getTarget();
        if (target == null) {
            return;
        }
        // for Chassis Speeds, +vx is foreward, +vy is left, and +omegaRadians is
        // counterclockwise

        // yaw is positive to the right (the tag is to the right relative to the camera)
        double yaw = -target.getYaw();
        double varianceInYaw = wantedYaw - yaw;
        // skew is positive when counter clockwise rotation relative to the camera
        double skew = -target.getSkew();
        double varianceInSkew = wantedSkew - skew;
        // pitch is positive when it is upwards relative to the camera
        double pitch = -target.getPitch();
        // distance increases as pitch decreases and vice versa
        double distanceToTarget = -Maths.DistanceFromTarget(pitch);
        double varianceInDistance = wantedDistance - distanceToTarget;

        double vx = 0;
        double vy = 0;
        double susanSpin = 0;
        if (varianceInSkew < -Constants.PHOTON_TOLERANCE_VALUE) {
            susanSpin = -0.25;
        } else if (varianceInSkew > Constants.PHOTON_TOLERANCE_VALUE) {
            susanSpin = 0.25;
        }

        if (varianceInYaw < -Constants.PHOTON_TOLERANCE_VALUE) {
            vy = -0.5;
        } else if (varianceInYaw > Constants.PHOTON_TOLERANCE_VALUE) {
            vy = 0.5;
        }

        if (varianceInDistance < -Constants.PHOTON_TOLERANCE_VALUE) {
            vx = 0.5;
        } else if (varianceInDistance > Constants.PHOTON_TOLERANCE_VALUE) {
            vx = -0.5;
        }

        System.out.println("center susan");

        lazySusan.spinSusan(susanSpin);
        driveTrain.drive(new ChassisSpeeds(vx, vy, 0));

    }
}
