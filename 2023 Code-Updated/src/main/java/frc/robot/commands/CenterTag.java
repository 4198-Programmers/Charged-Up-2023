package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class CenterTag extends CommandBase{
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double yaw;
    private boolean isfinished = false;
    
    public CenterTag(PhotonVision visionSub, DriveTrain swerveDriveSub) {
        super();
        this.vision = visionSub;
        this.swerveDrive = swerveDriveSub;
        addRequirements(visionSub, swerveDriveSub);
    }

    @Override
    public void execute() {
        this.yaw = vision.getYaw();
        if(Constants.WANTED_YAW < this.yaw) {
            swerveDrive.Move(-90, 0.5);
        } else if (Constants.WANTED_YAW > this.yaw) {
            swerveDrive.Move(90, 0.5);
        } else if (Constants.WANTED_YAW - 0.5 < this.yaw && Constants.WANTED_YAW + 0.5 > this.yaw) {
            this.isfinished = true;
        }
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return isfinished;
    }
}
