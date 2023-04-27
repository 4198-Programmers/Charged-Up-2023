package frc.robot.Tags;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Tags.PhotonVision;

public class TagFollowerExample extends CommandBase {
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    LazySusanSub lazySusanSub;
    private double wantedSpin;
    private double wantedTranslation;
    private double wantedDistance;
    PhotonTrackedTarget target;
    private boolean isFinished;
    double vx = 0;
    double vy = 0;
    double omegaRadians = 0;
    double photonDistance;
    double cameraAngle;
    double cameraTranslation;
    double gyroRotation;
    double wantedDriveAngle;
    double vySpeedSet;
    double vxSpeedSet;
    double vzSpeedSet;

    public TagFollowerExample(PhotonVision visionSub, DriveTrain swerveDriveSub, LazySusanSub lazySusanSub,
            double wantedSpin,
            double wantedTranslation,
            double wantedDistance, double wantedDriveAngle) {
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        this.wantedSpin = wantedSpin;
        this.wantedTranslation = wantedTranslation;
        this.wantedDistance = wantedDistance;
        this.wantedDriveAngle = wantedDriveAngle;
        this.lazySusanSub = lazySusanSub;
        addRequirements(visionSub, swerveDriveSub, lazySusanSub);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        target = vision.getTarget();

        if (target == null) {
            isFinished = true;
            return;
        }
        // photon vision's distance math didn't work

        photonDistance = Maths.distanceFromTargetBasedArea(target.getArea());
        //consistently off my about 3.5 inches
        cameraAngle = target.getSkew();
        cameraTranslation = target.getYaw() / 2.5168; // 1 unit in Yaw is about 2.5 inches
        gyroRotation = swerveDrive.getYaw();
        vySpeedSet = 0.75 * Maths.distancePhotonSpeedCalc(Math.abs(wantedDistance - photonDistance));
        vxSpeedSet = 1.25 * Maths.translateAndSpinPhotonSpeedCalc(Math.abs(wantedTranslation - cameraTranslation));
        vzSpeedSet = Maths.translateAndSpinPhotonSpeedCalc(Math.abs(wantedSpin - gyroRotation));
        // adjusting equation isn't small enough, just trying to get functional
        // absolute for an easier equation, direction set later

        if (photonDistance > (wantedDistance + 1)) {// 1 for tolerance
            // vy = -0.5;
            vy = -vySpeedSet;
        } else if (photonDistance < (wantedDistance - 1)) {
            // vy = 0.5;
            vy = vySpeedSet;
        } else {
            vy = 0;
        }

        // if (cameraAngle > (wantedSpin + 0.17453)) {
        // omegaRadians = 0.5;
        // } else if (cameraAngle < (wantedSpin - 0.17453)) {
        // omegaRadians = -0.5;
        // } else {
        // omegaRadians = 0;
        // } //get skew seems to not be working, forcing my hand

        if (gyroRotation < wantedDriveAngle - 2) {
            omegaRadians = vzSpeedSet;
        } else if (gyroRotation > wantedDriveAngle + 2) {
            omegaRadians = -vzSpeedSet;
        } else {
            omegaRadians = 0;
        }

        if (lazySusanSub.getLocation() < 1.95 && lazySusanSub.getLocation() <= 80) {
            // values have 0.05 tolerance, center of camera is 2 degrees off of robot arm
            // center
            lazySusanSub.susanEquationSpin(0, Constants.AUTO_SUSAN_SPEED);
        } else if (lazySusanSub.getLocation() > 2.05 && lazySusanSub.getLocation() >= -80) {
            lazySusanSub.susanEquationSpin(0, -Constants.AUTO_SUSAN_SPEED);
        } else {
            lazySusanSub.spinSusan(0);
        }

        if (cameraTranslation > (wantedTranslation + 1)) {// 2 for tolerance
            vx = -vxSpeedSet;
        } else if (cameraTranslation < (wantedTranslation - 1)) {
            vx = vxSpeedSet;
        } else {
            vx = 0;
        }

        System.out.println("speed " + vxSpeedSet);

        System.out.println(wantedTranslation + " Wanted");
        System.out.println(cameraTranslation + " Translation");

        swerveDrive
                .drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadians, swerveDrive.getGyroRotation(true)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
