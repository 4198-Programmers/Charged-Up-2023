package frc.robot.SwerveLib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
