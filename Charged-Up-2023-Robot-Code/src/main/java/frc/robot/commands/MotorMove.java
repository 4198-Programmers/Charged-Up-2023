package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Motor;

public class MotorMove extends CommandBase{
    double speed;
    double motorID;
    Motor motor;
    public MotorMove(double speed, double motorID, Motor motor){
        this.speed = speed;
        this.motorID = motorID;
        this.motor = motor;
        addRequirements(motor);
    }
    @Override
    public void initialize() {
        motor.getMotor(motorID);
    }
    @Override
    public void execute() {
        motor.setMotor(speed);
    }
}
