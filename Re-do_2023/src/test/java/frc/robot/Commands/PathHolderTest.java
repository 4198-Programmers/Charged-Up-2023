package frc.robot.Commands;

import static org.mockito.Mockito.mock;

import org.junit.jupiter.api.Test;

import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.PathHolder;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.VertArm;

public class PathHolderTest {
    @Test
    public void markers(){
        VertArm vertArm = mock(VertArm.class);
        Pneumatics pneumatics = mock(Pneumatics.class);
        ReachArmSub reachArmSub = mock(ReachArmSub.class);
        LazySusanSub lazySusanSub = mock(LazySusanSub.class);
        Swerve swerve = mock(Swerve.class);
        PathHolder pathHolder = new PathHolder(vertArm, pneumatics, reachArmSub, lazySusanSub, swerve);
    }
}
