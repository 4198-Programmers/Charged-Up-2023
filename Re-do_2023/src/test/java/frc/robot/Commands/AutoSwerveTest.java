package frc.robot.Commands;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PathHolder;

public class AutoSwerveTest {
    @Test
    public void autoSwerve(){
        PathHolder pathHolder = mock(PathHolder.class);
        Command command = mock(Command.class);
        when(pathHolder.createAutoPath()).thenReturn(command);
        AutoSwerve autoSwerve = new AutoSwerve(pathHolder);

        verify(pathHolder).createAutoPath();
    }
}
