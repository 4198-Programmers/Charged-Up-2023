package frc.robot.Commands;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.PathHolder;

public class FollowPathWithMarks extends CommandBase{
    PathHolder holder;
    DriveTrain driveTrain;

    public FollowPathWithMarks(PathHolder holder, DriveTrain driveTrain){
        this.holder = holder;
        this.driveTrain = driveTrain;
        addRequirements(holder, driveTrain);
    }

    @Override
    public void execute() {
        FollowPathWithEvents events = new FollowPathWithEvents(new RunPathAuto(holder, driveTrain), holder.getPathMarks(), holder.getPathEventMap());
        events.execute();
    }
    
}
