package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MathDriveTrain;

public class MathDriveCom extends CommandBase {
    MathDriveTrain driveTrain;
    Joystick joystickOne;
    Joystick joystickTwo;

    public MathDriveCom(MathDriveTrain driveArg, Joystick joystickOneArg, Joystick joystickTwoArg) {
        driveTrain = driveArg;
        joystickOne = joystickOneArg;
        joystickTwo = joystickTwoArg;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double x = joystickOne.getX();
        double y = joystickOne.getY();
        double z = joystickTwo.getZ();
        driveTrain.drive(x,y,z);
    }

}
