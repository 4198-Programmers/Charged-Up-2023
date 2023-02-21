package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveDriveBase;

public class Balance extends CommandBase{
    SwerveDriveBase swerveDriveBase;
    ChassisSpeeds chassisSpeeds;

    public Balance(SwerveDriveBase swerveDriveBase){
        this.swerveDriveBase = swerveDriveBase;
        addRequirements(swerveDriveBase);
    }
    @Override
    public void initialize() {
        chassisSpeeds = new ChassisSpeeds(0, Constants.BALANCE_SPEED, 0);
    }
    @Override
    public void execute() {
        SwerveModuleState[] moduleStates = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveDriveBase.setModuleStates(moduleStates);
    }
    @Override
    public boolean isFinished() {
        return swerveDriveBase.BalanceDrive() == 0;
    }
}
