package frc.robot.SwerveDrive.Controllers.SwerveController;
/**Gets values from the steer controller */
public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
}
