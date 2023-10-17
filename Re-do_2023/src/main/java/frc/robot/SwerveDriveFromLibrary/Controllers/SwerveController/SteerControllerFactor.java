package frc.robot.SwerveDriveFromLibrary.Controllers.SwerveController;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.SwerveDriveFromLibrary.ModuleConfiguration;

@FunctionalInterface
/**Makes the Steer Controller */
public interface SteerControllerFactor <Controller extends SteerController, SteerConfiguration>{
    /**Puts the steer angles into the shuffleboard dashboard */
    default void addDashboardEntries(ShuffleboardContainer container, Controller controller){
        container.addNumber("Current Angle", () -> Math.toDegrees(controller.getStateAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(controller.getReferenceAngle()));
    }
/** Creates a controller using the steer and module configuration */
    default Controller create(
        ShuffleboardContainer dashboardContainer, 
        SteerConfiguration steerConfiguration, 
        ModuleConfiguration moduleConfiguration){
            var controller = create(steerConfiguration, moduleConfiguration);
            addDashboardEntries(dashboardContainer, controller);

            return controller;
    }

    Controller create(SteerConfiguration steerConfiguration, ModuleConfiguration moduleConfiguration);
}
