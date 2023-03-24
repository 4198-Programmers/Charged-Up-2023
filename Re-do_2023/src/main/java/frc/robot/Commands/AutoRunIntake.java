package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;

public class AutoRunIntake extends CommandBase {
    Intake intake;
    double speed;
    long timeStart;
    boolean isFinished;

    public AutoRunIntake(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        isFinished = false;
        timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - timeStart < 500) {
            intake.SetSpeed(speed);
        } else {
            intake.SetSpeed(0);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        intake.SetSpeed(0);
    }
}
