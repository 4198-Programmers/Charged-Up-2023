package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.PhotonVision;

public class CheckPhotonTarget extends CommandBase{
    private PhotonVision vision; 
    public CheckPhotonTarget(PhotonVision photonVision) {
        super();
        this.vision = photonVision;
        addRequirements(photonVision);
    }

    @Override
    public void execute() {
        this.vision.getBestTarget();
    }


}
