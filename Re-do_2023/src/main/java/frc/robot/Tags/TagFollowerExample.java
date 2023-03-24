package frc.robot.Tags;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Tags.PhotonVision;

public class TagFollowerExample extends CommandBase {
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double wantedSpin;
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

    public TagFollowerExample(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedSpin, double wantedSkew,
            double wantedDistance) {
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedSpin = wantedSpin;
        this.wantedSkew = wantedSkew;
        this.goalRange = Units.inchesToMeters(wantedDistance);
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

        if (target == null) {
            xFinished = true;
            yFinished = true;
            zFinished = true;
            return;
        }
        // for Chassis Speeds, +vx is foreward, +vy is right, and +omegaRadians is
        // counterclockwise

        // yaw is positive to the right (the tag is to the right relative to the camera)
        double yaw = target.getYaw();
        // skew is positive when counter clockwise rotation relative to the camera
        double skew = target.getSkew();
        // pitch is positive when it is upwards relative to the camera
        double pitch = target.getPitch();
        // distance increases as pitch decreases and vice versa

        if (wantedSkew < -0.5) {
        vy = -0.5;
        } else if (wantedSkew > 0.5) {
        vy = 0.5;
        } else {
        vy = 0;
        yFinished = true;
        }

        if (wantedDistance < -0.5) {
        vx = -0.5;
        } else if (wantedDistance > 0.5) {
        vx = 0.5;
        } else {
        vx = 0;
        xFinished = true;
        }

        if (yaw < wantedSpin-0.5) {
            omegaRadians = -0.5;
        } else if (yaw > 0.5) {
            omegaRadians = 0.5;
        } else {
            omegaRadians = 0;
            zFinished = true;
        }

        swerveDrive
                .drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadians, swerveDrive.getGyroRotation(true)));
    }

    @Override
    public boolean isFinished() {
        return zFinished && yFinished && xFinished;
    }

}
