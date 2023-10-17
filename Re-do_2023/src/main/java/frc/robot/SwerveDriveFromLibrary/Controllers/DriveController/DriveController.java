package frc.robot.SwerveDriveFromLibrary.Controllers.DriveController;
/**Gets values from the drive controller */
public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
