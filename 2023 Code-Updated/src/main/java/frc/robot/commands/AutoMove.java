// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class AutoMove extends CommandBase {
//     MathDriveTrain driveTrain;
//     double wantedAngle;
//     double wantedSpeed;

//     public AutoMove(MathDriveTrain driveTrainArg, double angleArg, double speedArg){
//         driveTrain = driveTrainArg;
//         wantedAngle = angleArg;
//         wantedSpeed = speedArg;
//         addRequirements(driveTrain);
//     }

//     @Override
//     public void initialize() {
//         //reset drive position or gyro
//     }

//     @Override
//     public void execute() {
//         driveTrain.AutoDrive(wantedAngle, wantedSpeed);
//     }

//     @Override
//     public boolean isFinished() {
//         return true;//should be comparing current position to goal enc position 
//     }


// }
