package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class ZeroHeading extends CommandBase{
    DriveTrain driveTrain;
    boolean stopLoopCondition;
    boolean done;

    public ZeroHeading(DriveTrain driveTrain, boolean stopLoopCondition){
        this.driveTrain = driveTrain;
        this.stopLoopCondition = stopLoopCondition;
    }
    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        System.out.println("Stop Loop: " + stopLoopCondition);
        if(stopLoopCondition == false){
            driveTrain.getRobotOrientationRotation();
            System.out.println("Robot Orientation");

        }else if(stopLoopCondition == true){
        driveTrain.zeroGyro(); 
        System.out.println("Field Orientation");
        }
        done = stopLoopCondition;
    }
    @Override
    public boolean isFinished() {
        return done;
    }

}
