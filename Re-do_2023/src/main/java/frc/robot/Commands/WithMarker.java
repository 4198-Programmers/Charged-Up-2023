package frc.robot.Commands;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.PathHolder;

public class WithMarker extends CommandBase {
    DriveTrain driveTrain;
    PathHolder pathHolder;
    FollowPathWithEvents command;

    public WithMarker(DriveTrain driveTrain, PathHolder pathHolder) {
        this.driveTrain = driveTrain;
        this.pathHolder = pathHolder;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        command = new FollowPathWithEvents(new RunPathAuto(pathHolder, driveTrain), pathHolder.getPathMarks(),
                pathHolder.getPathEventMap());
    }

    @Override
    public void execute() {
        command.execute();
    }

}
