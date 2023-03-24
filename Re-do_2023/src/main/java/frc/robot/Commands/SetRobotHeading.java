package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class SetRobotHeading extends CommandBase {
    DriveTrain driveTrain;
    boolean done;
    float gyroRotation;
    double wantedDegrees;

    public SetRobotHeading(DriveTrain driveTrain, double wantedDegrees) {
        this.driveTrain = driveTrain;
        this.wantedDegrees = wantedDegrees;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        gyroRotation = driveTrain.getYaw(); 

        System.out.println("straightening");
        if (gyroRotation < wantedDegrees - 3) {
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1, 
            driveTrain.getGyroRotation(true)));
        } else if(gyroRotation > wantedDegrees + 3){
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -1, 
            driveTrain.getGyroRotation(true)));
        } else{
            driveTrain.StopDrive();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
