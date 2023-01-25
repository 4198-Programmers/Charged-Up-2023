package frc.robot.commands;

import org.opencv.photo.Photo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class FlattenTag extends CommandBase{
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private Boolean isFinished;
    private double skew;

    public FlattenTag(PhotonVision visionSub, DriveTrain swerveDriveSub) {
        super();
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        addRequirements(visionSub, swerveDriveSub);
    }
    
    @Override
    public void execute() {
        this.skew = vision.getSkew();
        if(Constants.WANTED_SKEW < this.skew) {
            swerveDrive.Move(120, 0.5);
        } else if (Constants.WANTED_SKEW > this.skew) {
            swerveDrive.Move(-120, 0.5);
        } else if (Constants.WANTED_SKEW - 1 < this.skew && Constants.WANTED_SKEW + 1 > this.skew) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
