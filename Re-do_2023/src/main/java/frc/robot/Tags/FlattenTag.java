package frc.robot.Tags;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveTrain;

public class FlattenTag extends CommandBase {
    // center tag via spin, to make appear flat to camera
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private Boolean isFinished;
    double wantedSkew;
    double skew;
    double rotationToWanted;
    double timeEnded;

    public FlattenTag(PhotonVision visionSub, DriveTrain swerveDriveSub, double wantedSkew) {
        this.wantedSkew = wantedSkew;
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = vision.getTarget();
        if (target == null) {
            timeEnded = System.currentTimeMillis();
            isFinished = true;
            return;
        } else {
            skew = target.getSkew();
            rotationToWanted = Constants.WANTED_SKEW_MID - skew;
            if (rotationToWanted < -Constants.PHOTON_TOLERANCE_VALUE) {
                timeEnded = -1;
                swerveDrive.drive(new ChassisSpeeds(0, 0, 0.5));
                System.out.println("Skew Difference: " + rotationToWanted);
            } else if (rotationToWanted > Constants.PHOTON_TOLERANCE_VALUE) {
                timeEnded = -1;
                swerveDrive.drive(new ChassisSpeeds(0, 0, -0.5));
            } else {
                isFinished = true;
                System.out.println("Flatten Finished " + isFinished);
                if (timeEnded == -1) {
                    timeEnded = System.currentTimeMillis();
                }

            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
