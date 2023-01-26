package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LazySusanSub;

public class ControlSusan extends CommandBase{
    LazySusanSub lazySusan;
    Joystick joystick;
    
    public ControlSusan(LazySusanSub susanArg, Joystick joystickRight){
        lazySusan = susanArg;
        joystick = joystickRight;
        addRequirements(lazySusan);
    }

    @Override
    public void execute() {
        lazySusan.spinSusan(joystick.getX());
    }
}