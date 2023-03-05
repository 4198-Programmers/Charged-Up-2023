package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorTesting;

public class MotorTestingCom extends CommandBase{
    MotorTesting motor;
    Joystick stick;

    public MotorTestingCom(MotorTesting motorTestingArg, Joystick stickArg){
        motor = motorTestingArg;
        stick = stickArg;
        addRequirements(motor);
    }

    @Override
    public void execute() {
        motor.go(-stick.getY());
        
    }
    
}
