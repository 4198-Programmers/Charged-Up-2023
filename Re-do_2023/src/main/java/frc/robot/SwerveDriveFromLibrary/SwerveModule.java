package frc.robot.SwerveDriveFromLibrary;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);
}
