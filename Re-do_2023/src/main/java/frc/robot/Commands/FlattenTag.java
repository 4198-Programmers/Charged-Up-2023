// package frc.robot.Commands;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Subsystems.DriveTrain;
// import frc.robot.Subsystems.PhotonVision;

// public class FlattenTag extends CommandBase{
//     private PhotonVision vision;
//     private DriveTrain swerveDrive;
//     private Boolean isFinished;


//     public FlattenTag(PhotonVision visionSub, DriveTrain swerveDriveSub) {
//         super();
//         this.vision = visionSub;
//         this.swerveDrive = swerveDriveSub;
//         addRequirements(visionSub, swerveDriveSub);
//     }
    
//     @Override
//     public void execute() {
//         PhotonTrackedTarget target = this.vision.getBestTarget();
//         if(target == null) {
//             this.isFinished = true;
//             return;
//         }
//         double skew = target.getSkew();
//         double varianceInSkew = Constants.WANTED_SKEW_MID - skew;
//         if(varianceInSkew < -0.5) {
//             swerveDrive.drive(new ChassisSpeeds(0,0,-0.5));;
//         } else if (varianceInSkew > 0.5) {
//             swerveDrive.drive(new ChassisSpeeds(0,0,0.5));
//         } else {
//             this.isFinished = true;
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return isFinished;
//     }
// }
