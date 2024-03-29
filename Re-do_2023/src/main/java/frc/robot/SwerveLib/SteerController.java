package frc.robot.SwerveLib;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    double getAbsoluteCanCoderAngle();

    double configMotorOffset(boolean logErrors);

}
