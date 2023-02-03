package frc.robot.wpiVcontainer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FieldDriveCom extends CommandBase {
    FieldDriveSub driveTrain;
    Joystick joystick;
    Joystick joystickTwo;
    double xAxis;
    double yAxis;
    double zAxis;
    double maxMPS = 0.4667056958;

    public FieldDriveCom(FieldDriveSub driveArg, Joystick joystickArg, Joystick joystickTwoArg) {
        driveTrain = driveArg;
        joystick = joystickArg;
        joystickTwo = joystickTwoArg;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.reset();
    }

    @Override
    public void execute() {
        xAxis = deadband(joystick.getX(), 0.1) * maxMPS;
        yAxis = deadband(joystick.getY(), 0.1) * maxMPS;
        zAxis = deadband(joystickTwo.getX(), 0.1) * maxMPS;//controller change is done later in code
        driveTrain.drive(xAxis, yAxis, zAxis, false);
    }

    private double deadband(double current, double limit) {
        return current;
        // if(Math.abs(current) < limit){
        //     return 0;
        // } 
        // double max = 1 - limit;
        // double speed = (Math.abs(current) - limit / max) * (current / Math.abs(current));
        // return speed;
    }

}
