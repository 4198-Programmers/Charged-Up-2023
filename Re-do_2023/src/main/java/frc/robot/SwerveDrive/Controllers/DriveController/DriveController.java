package frc.robot.SwerveDrive.Controllers.DriveController;
/**Gets values from the drive controller */
public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
