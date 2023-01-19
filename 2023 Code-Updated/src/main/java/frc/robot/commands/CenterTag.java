package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class CenterTag extends CommandBase {
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    public CenterTag(PhotonVision photonVision) {
        super();
        this.vision = photonVision;
        addRequirements(photonVision);
    }
    @Override
    public void execute() {
        if(vision.getPitch() < Constants.WANTED_PITCH) {
            
        }
        super.execute();
    }
}
