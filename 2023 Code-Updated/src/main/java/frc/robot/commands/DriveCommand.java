package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainW;

public class DriveCommand extends CommandBase{
    DriveTrainW drive;
    Joystick joystick;

    public DriveCommand(DriveTrainW driveArg, Joystick joystickArg){
        this.drive = driveArg;
        this.joystick = joystickArg;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.reset();
    }

    @Override
    public void execute() {
        drive.drive(joystick.getRawAxis(0), joystick.getRawAxis(1), joystick.getRawAxis(2), false);
    }
    
}
