package frc.robot.commands;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class FlattenTag extends CommandBase{
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private Boolean isFinished;


    public FlattenTag(PhotonVision visionSub, DriveTrain swerveDriveSub) {
        super();
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        addRequirements(visionSub, swerveDriveSub);
    }
    
    @Override
    public void execute() {
        PhotonTrackedTarget target = this.vision.getBestTarget();
        if(target == null) {
            this.isFinished = true;
            return;
        }
        double skew = target.getSkew();
        double varianceInSkew = Constants.WANTED_SKEW - skew;
        if(varianceInSkew < -0.5) {
            swerveDrive.Move(120, 0.5);
        } else if (varianceInSkew > 0.5) {
            swerveDrive.Move(-120, 0.5);
        } else {
            this.isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
