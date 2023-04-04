package frc.robot.Tags;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;

public class CPTagDrive extends CommandBase {

    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double wantedYaw;
    private double wantedSkew;
    private double wantedDistance;
    PhotonTrackedTarget target;
    private boolean isFinished;
    double vx;
    double vy;
    double omegaRadians;
    final double cameraHeightMeters = Units.inchesToMeters(18.35);
    final double targetHeightMeters = Units.inchesToMeters(18.25);
    final double cameraPitchRadians = Units.degreesToRadians(0);
    double goalRange;

    public CPTagDrive(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedYaw, double wantedSkew,
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
        isFinished = false;
        vx = 0;
        vy = 0;
        omegaRadians = 0;
    }

    @Override
    public void execute() {
        target = vision.getTarget();

        if (target == null) {
            isFinished = true;
            return;
        }
        // for Chassis Speeds, +vx is foreward, +vy is right, and +omegaRadians is
        // counterclockwise

        double yaw = target.getYaw();
        double varianceInYaw = Math.abs(wantedYaw) - yaw;
        vx = -(-0.000366767 * Math.pow(varianceInYaw, 3)) + (0.271014 * varianceInYaw);
        // skew is positive when counter clockwise rotation relative to the camera
        double skew = -target.getSkew();
        double varianceInSkew = Math.abs(wantedSkew) - skew;
        omegaRadians = (0.0000237793 * Math.pow(varianceInSkew, 3)) + (0.0455666 * varianceInSkew);
        // pitch is positive when it is upwards relative to the camera
        double pitch = -target.getPitch();
        // distance increases as pitch decreases and vice versa
        double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters,
                cameraPitchRadians, Units.degreesToRadians(target.getPitch()));
        double varianceInDistance = Math.abs(goalRange) - range;
        vy = (-0.000366767 * Math.pow(varianceInDistance, 3)) + (0.271014 * varianceInDistance);

        String printValues = String.format("%.2f,%.2f,%.2f", yaw, skew, pitch);
        System.out.println("yaw, skew, pitch ");
        System.out.println(printValues);
        swerveDrive
                .drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadians, swerveDrive.getGyroRotation(true)));
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(vx) < 0.05) && (Math.abs(vy) < 0.05) && (Math.abs(omegaRadians) < 0.05);
    }

}
