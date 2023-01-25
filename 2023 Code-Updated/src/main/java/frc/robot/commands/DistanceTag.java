package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Maths;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class DistanceTag extends CommandBase {
    private PhotonVision vision;
    private DriveTrain swerveDrive;
    private double distanceToTarget;
    private boolean isfinished = false;
    public DistanceTag(PhotonVision photonVision, DriveTrain swerveDriveTrain) {
        super();
        this.vision = photonVision;
        this.swerveDrive = swerveDriveTrain;
        addRequirements(photonVision, swerveDriveTrain);
    }
    @Override
    public void execute() {
        this.distanceToTarget = Maths.DistanceFromTarget(vision.getPitch());
        if(Constants.WANTED_DISTANCE < this.distanceToTarget) {
            swerveDrive.Move(0, 0.5);
        } else if(Constants.WANTED_DISTANCE > this.distanceToTarget) {
            swerveDrive.Move(180, 0.5);
        } else if (Constants.WANTED_DISTANCE + 0.5 > this.distanceToTarget && Constants.WANTED_DISTANCE - 0.5 < this.distanceToTarget) {
            this.isfinished = true;
        }
    
    }
    @Override
    public boolean isFinished() {
        return this.isfinished;
    }
}
