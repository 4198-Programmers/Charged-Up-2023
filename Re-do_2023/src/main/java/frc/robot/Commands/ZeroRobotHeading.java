package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ZeroRobotHeading extends CommandBase {
    DriveTrain driveTrain;
    boolean done;
    float gyroRotation;

    public ZeroRobotHeading(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        gyroRotation = driveTrain.getYaw(); 

        System.out.println("straightening");
        if (gyroRotation < -3) {
            driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1, 
            driveTrain.getGyroRotation(true)));
        } else if(gyroRotation > 3){
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
