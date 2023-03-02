package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve;

public class BalanceCommand extends CommandBase{
    Swerve swerve;

    public BalanceCommand(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }
    @Override
    public void execute() {
        swerve.drive(swerve.balance(), 0, 0, false);
    }
    @Override
    public boolean isFinished() {
        return swerve.balance() == 0;
    }
}
