package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.PathHolder;
import frc.robot.Subsystems.Swerve;

public class AutoSwerve extends SequentialCommandGroup{
    public AutoSwerve(Swerve s_Swerve, String pathName, PathHolder pathHolder){
        addCommands(
        new InstantCommand(() ->pathHolder.createAutoPath()));
    }
}
