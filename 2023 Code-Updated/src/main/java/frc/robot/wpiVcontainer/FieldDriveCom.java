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
        xAxis = joystick.getX() * maxMPS;
        yAxis = joystick.getY() * maxMPS;
        zAxis = joystickTwo.getX() * maxMPS;//controller change is done later in code
        driveTrain.drive(xAxis, yAxis, zAxis, false);
    }

}
