package frc.robot.Commands;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;

import frc.robot.Subsystems.VertArm;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
@TestInstance(value = Lifecycle.PER_CLASS)
public class ControlVertArmTest {
    //private CommandScheduler scheduler = null;

    // @BeforeEach
    //     void setup(){
    //         // assert HAL.initialize(500, 0);
    //         scheduler = CommandScheduler.getInstance();
    //         // vertArm = mock(VertArm.class);
    //         // controlArm = new ControlArm(vertArm, () -> 1, 100);
            
    //     }
    @Test
    public void testMoveUp(){
        //Arrange: Set up dependency and itself
        VertArm vertArm = mock(VertArm.class);
        ControlArm controlArm = new ControlArm(vertArm, () -> 1, 100);
        controlArm.initialize();
        controlArm.execute();

        //Act
        // scheduler.schedule(controlArm);
        // scheduler.run();
        // scheduler.setActiveButtonLoop(new EventLoop());

        //Assert
        verify(vertArm).moveArm(1);
    }
    @Test
    public void testMoveDown(){
        //Arrange: Set up dependency and itself
        VertArm vertArm = mock(VertArm.class);
        ControlArm controlArm = new ControlArm(vertArm, () -> -1, 100);
        controlArm.initialize();
        controlArm.execute();

        //Act
        // scheduler.schedule(controlArm);
        // scheduler.run();
        // scheduler.setActiveButtonLoop(new EventLoop());

        //Assert
        verify(vertArm).moveArm(-1);
    }
    // @AfterAll
    // public  void cleanup(){
    //     scheduler.cancelAll();
    // }

}
