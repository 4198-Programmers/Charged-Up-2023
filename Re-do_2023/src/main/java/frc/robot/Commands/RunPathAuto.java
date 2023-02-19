package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.PathHolder;

public class RunPathAuto extends CommandBase {
    private final PathHolder path;
    private final DriveTrain driveTrain;
    // private long timeStart;
    private double matchTime;
    private ChassisSpeeds toSwerveSpeeds;
    private double[] velocitiesHolder;

    public RunPathAuto(PathHolder pathSub, DriveTrain driveArg) {
        this.path = pathSub;
        this.driveTrain = driveArg;
        addRequirements(pathSub, driveArg);
    }

    @Override
    public void initialize() {
        // timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // matchTime = (double) (System.currentTimeMillis() - timeStart);
        matchTime = (15 - DriverStation.getMatchTime());
        // gives match time countdown not up, subtract from 15 to be accurate
        velocitiesHolder = path.getPathVelocities(matchTime);
        toSwerveSpeeds = new ChassisSpeeds(velocitiesHolder[0], velocitiesHolder[1], velocitiesHolder[2]);
        driveTrain.drive(toSwerveSpeeds);
    }

}
