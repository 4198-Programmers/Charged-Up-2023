package frc.robot.wpiVcontainer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    DriveTrainW driveTrain;
    Joystick joystick;
    double xAxis;
    double yAxis;
    double zAxis;
    double maxMPS = 0.4667056958;

    public DriveCommand(DriveTrainW driveArg, Joystick joystickArg) {
        driveTrain = driveArg;
        joystick = joystickArg;
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
        zAxis = joystick.getZ() * maxMPS;
        driveTrain.drive(xAxis, yAxis, zAxis, false);
    }

}
