package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;

public class BalanceCommand extends CommandBase{
    Swerve swerve;

    public BalanceCommand(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }
    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(swerve.balance(), 0, 0);
        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);

    }
    @Override
    public boolean isFinished() {
        return swerve.balance() == 0;
    }
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }
}
