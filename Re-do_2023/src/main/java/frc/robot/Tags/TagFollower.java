package frc.robot.Tags;

import java.util.Objects;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Tags.PhotonVision;

public class TagFollower extends CommandBase {
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double wantedYaw;
    private double wantedSkew;
    private double wantedDistance;
    PhotonTrackedTarget target;
    private boolean xFinished;
    private boolean yFinished;
    private boolean zFinished;
    double vx = 0;
    double vy = 0;
    double omegaRadians = 0;
    double timeXEnded;
    double timeYEnded;
    double timeZEnded;

    public TagFollower(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedYaw, double wantedSkew,
            double wantedDistance) {
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedYaw = wantedYaw;
        this.wantedSkew = wantedSkew;
        this.wantedDistance = wantedDistance;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void initialize() {
        xFinished = false;
        yFinished = false;
        zFinished = false;
        vx = 0;
        vy = 0;
        omegaRadians = 0;
    }

    @Override
    public void execute() {
        target = vision.getTarget();

        if (target == null) {
            System.out.println("No Tag");
            xFinished = true;
            yFinished = true;
            zFinished = true;
            timeXEnded = System.currentTimeMillis();
            timeYEnded = System.currentTimeMillis();
            timeZEnded = System.currentTimeMillis();
            return;
        } else {
            // for Chassis Speeds, +vx is foreward, +vy is right, and +omegaRadians is
            // counterclockwise

            // yaw is positive to the right (the tag is to the right relative to the camera)
            double yaw = target.getYaw();
            double varianceInYaw = wantedYaw - yaw;
            // skew is positive when counter clockwise rotation relative to the camera
            double skew = target.getSkew();
            double varianceInSkew = wantedSkew - skew;
            // pitch is positive when it is upwards relative to the camera
            double pitch = target.getPitch();
            // distance increases as pitch decreases and vice versa
            double distanceToTarget = (PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT,
                    Constants.APRIL_TAG_HEIGHT, 0, Units.degreesToRadians(pitch))) * -1;
            // Maths.DistanceFromTarget(pitch);
            double varianceInDistance = wantedDistance - distanceToTarget;
            int basis = 1;

            switch (basis) {
                case 1:
                    System.out.println("");
                    break;
            }

            if (varianceInSkew < -0.5) {
                System.out.println("Skew Variance: " + varianceInSkew);
                zFinished = false;
                timeZEnded = -1;
                omegaRadians = 0.5;
            } else if (varianceInSkew > 0.5) {
                System.out.println("Skew Variance: " + varianceInSkew);
                zFinished = false;
                timeZEnded = -1;
                omegaRadians = -0.5;
            } else {
                System.out.println("Skew Variance: " + varianceInSkew);
                System.out.println("Skew Finished");
                omegaRadians = 0;
                zFinished = true;
                if (timeZEnded == -1) {
                    this.timeZEnded = System.currentTimeMillis();
                }
            }

            if (varianceInYaw < -4) {
                System.out.println("Yaw Variance: " + varianceInYaw);
                timeXEnded = -1;
                xFinished = false;
                vx = -0.5;
                System.out.println(vx);
            } else if (varianceInYaw > 4) {
                System.out.println("Yaw Variance: " + varianceInYaw);
                xFinished = false;
                timeXEnded = -1;
                vx = 0.5;
                System.out.println(vx);
            } else {
                System.out.println("Yaw Variance: " + varianceInYaw);
                System.out.println("Yaw Finished");
                vx = 0;
                xFinished = true;
                System.out.println(vx);
                if (timeYEnded == -1) {
                    this.timeXEnded = System.currentTimeMillis();
                }
            }

            if (varianceInDistance < -0.01) {
                System.out.println("Current Distance: " + distanceToTarget);
                System.out.println("Wanted Distance: " + wantedDistance);
                System.out.println("Pitch: " + pitch);
                System.out.println("Distance Variance: " + varianceInDistance);
                yFinished = false;
                timeYEnded = -1;
                vy = -0.5;
                System.out.println(vy);
            } else if (varianceInDistance > 0.01) {
                System.out.println("Current Distance" + distanceToTarget);
                System.out.println("Wanted Distance" + wantedDistance);
                System.out.println("Pitch: " + pitch);
                System.out.println("Distance Variance: " + varianceInDistance);
                yFinished = false;
                timeYEnded = -1;
                vy = 0.5;
                System.out.println(vy);
            } else {
                System.out.println("Current Distance" + distanceToTarget);
                System.out.println("Wanted Distance" + wantedDistance);
                System.out.println("Pitch: " + pitch);
                System.out.println("Distance Variance: " + varianceInDistance);
                System.out.println("Distance Finished");
                vy = 0;
                yFinished = true;
                System.out.println(vy);
                if (timeXEnded == -1) {
                    this.timeYEnded = System.currentTimeMillis();
                }
            }
            swerveDrive
                    .drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx * 0.5, vy * (Math.abs(varianceInDistance)), omegaRadians * 0.5,
                            swerveDrive.getGyroRotation(true)));
        }

    }

    @Override
    public boolean isFinished() {
        return zFinished && yFinished && xFinished && System.currentTimeMillis() > timeZEnded + 500
                && System.currentTimeMillis() > timeYEnded + 500 && System.currentTimeMillis() > timeXEnded + 500
                && timeXEnded > 0 && timeYEnded > 0 && timeZEnded > 0;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerveDrive.getGyroRotation(true)));
    }
}
