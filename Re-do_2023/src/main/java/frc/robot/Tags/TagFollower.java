package frc.robot.Tags;

import java.util.Objects;

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
    final double cameraHeightMeters = Units.inchesToMeters(19);
    final double targetHeightMeters = Units.inchesToMeters(18);
    final double cameraPitchRadians = Units.degreesToRadians(0);
    double goalRange;
    double timeXEnded;
    double timeYEnded;
    double timeZEnded;

    public TagFollower(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedYaw, double wantedSkew,
            double wantedDistance) {
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedYaw = wantedYaw;
        this.wantedSkew = wantedSkew;
        this.goalRange = Units.inchesToMeters(wantedDistance);
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
            double distanceToTarget = Maths.DistanceFromTarget(pitch);
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

            if (varianceInYaw < -0.5) {
                System.out.println("Yaw Variance: " + varianceInYaw);
                timeYEnded = -1;
                yFinished = false;
                vx = -0.5;
                System.out.println(vy);
            } else if (varianceInYaw > 0.5) {
                System.out.println("Yaw Variance: " + varianceInYaw);
                yFinished = false;
                timeYEnded = -1;
                vx = 0.5;
                System.out.println(vy);
            } else {
                System.out.println("Yaw Variance: " + varianceInYaw);
                System.out.println("Yaw Finished");
                vx = 0;
                yFinished = true;
                System.out.println(vy);
                if (timeYEnded == -1) {
                    this.timeYEnded = System.currentTimeMillis();
                }
            }

            if (varianceInDistance < -0.5) {
                System.out.println("Distance Variance: " + varianceInDistance);
                xFinished = false;
                timeXEnded = -1;
                vy = -0.5;
                System.out.println(vx);
            } else if (varianceInDistance > 0.5) {
                System.out.println("Distance Variance: " + varianceInDistance);
                xFinished = false;
                timeXEnded = -1;
                vy = 0.5;
                System.out.println(vx);
            } else {
                System.out.println("Distance Variance: " + varianceInDistance);
                System.out.println("Distance Finished");
                vy = 0;
                xFinished = true;
                System.out.println(vy);
                if (timeXEnded == -1) {
                    this.timeXEnded = System.currentTimeMillis();
                }
            }
            swerveDrive
                    .drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadians,
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
