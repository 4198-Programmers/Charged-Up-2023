package frc.robot.Tags;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CheckPhotonTarget extends CommandBase {
    // don't think we need this
    private PhotonVision vision;
    boolean isFinished;

    public CheckPhotonTarget(PhotonVision photonVision) {
        this.vision = photonVision;
        addRequirements(photonVision);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        vision.getTarget();
        System.out.println("Pitch: " + vision.getTarget().getPitch());
        System.out.println("Yaw: " + vision.getTarget().getYaw());
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
