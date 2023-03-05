// package frc.robot.Tags;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Maths;
// import frc.robot.Subsystems.DriveTrain;
// import frc.robot.Subsystems.PhotonVision;

// public class DistanceTag extends CommandBase {
//     private PhotonVision vision;
//     private DriveTrain swerveDrive;

//     private boolean isfinished = false;
//     public DistanceTag(PhotonVision photonVision, DriveTrain swerveDriveTrain) {
//         super();
//         this.vision = photonVision;
//         this.swerveDrive = swerveDriveTrain;
//         addRequirements(photonVision, swerveDriveTrain);
//     }
//     @Override
//     public void execute() {
//         PhotonTrackedTarget target = this.vision.getBestTarget();
//         if(target == null) {
//             this.isfinished = true;
//             return;
//         }
//         double distanceToTarget = Maths.DistanceFromTarget(vision.getPitch());
//         double varianceInDistance = Constants.WANTED_DISTANCE_MID - distanceToTarget;
//         if(varianceInDistance < -0.5) {
//             swerveDrive.drive(new ChassisSpeeds(-0.5, 0, 0));;
//         } else if(varianceInDistance > 0.5) {
//             swerveDrive.drive(new ChassisSpeeds(0.5, 0, 0));;
//         } else {
//             this.isfinished = true;
//         }
    
//     }
//     @Override
//     public boolean isFinished() {
//         return this.isfinished;
//     }
// }
