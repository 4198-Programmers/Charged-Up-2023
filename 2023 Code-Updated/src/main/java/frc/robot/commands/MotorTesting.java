package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MotorTesting extends CommandBase{
    CANSparkMax testMotor;
    Joystick joystick;

    public MotorTesting(Joystick stick, int motorID, MotorType type){
        testMotor = new CANSparkMax(motorID, type);
        joystick = stick;
    }

    @Override
    public void execute() {
        testMotor.set(joystick.getX());
        System.out.println("driving");
    }

    @Override
    public void end(boolean interrupted) {
        testMotor.set(0);
    }
    
}
