package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;

public class RunIntake extends CommandBase{
    Intake intake;
    double speed;

    public RunIntake(Intake intake, double speed){
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.SetSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.SetSpeed(0);
    }

    
}
