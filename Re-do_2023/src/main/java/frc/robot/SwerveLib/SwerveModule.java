package frc.robot.SwerveLib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void deadWheelCheckMotorOffsets();
}
