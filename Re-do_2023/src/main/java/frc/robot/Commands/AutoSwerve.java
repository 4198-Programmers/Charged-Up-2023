package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.PathHolder;

public class AutoSwerve extends SequentialCommandGroup{
    public AutoSwerve(PathHolder pathHolder){
        addCommands(pathHolder.createAutoPath());
    }
}
