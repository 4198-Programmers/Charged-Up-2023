package frc.robot.Commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.BeforeEach;

import frc.robot.Commands.ControlArm;
import frc.robot.Subsystems.VertArm;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class ControlVertArmTest {
    private CommandScheduler scheduler = null;

    @BeforeEach
        void setup(){
            // assert HAL.initialize(500, 0);
            scheduler = CommandScheduler.getInstance();
            // vertArm = mock(VertArm.class);
            // controlArm = new ControlArm(vertArm, () -> 1, 100);
            
        }
    @Test
    public void testMove(){
        //Arrange: Set up dependency and itself
        VertArm vertArm = mock(VertArm.class);
        ControlArm controlArm = new ControlArm(vertArm, () -> 1, 100);

        //Act
        scheduler.schedule(controlArm);
        scheduler.run();

        //Assert
        verify(vertArm).moveArm(() -> 1);
    }

}
