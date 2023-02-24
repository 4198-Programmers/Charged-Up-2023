package frc.robot.Commands;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.PathHolder;

public class RunPathAuto extends CommandBase {
    private final PathHolder path;
    private final DriveTrain driveTrain;
    private long timeStart;
    private double matchTime;
    private ChassisSpeeds toSwerveSpeeds;
    private double[] variablesHolder;

    public RunPathAuto(PathHolder pathSub, DriveTrain driveArg) {
        this.path = pathSub;
        this.driveTrain = driveArg;
        addRequirements(pathSub, driveArg);
    }

    @Override
    public void initialize() {
        timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        matchTime = (double) ((System.currentTimeMillis() - timeStart) / 1000);
        variablesHolder = path.getPathVelocities(matchTime);

        if (matchTime <= variablesHolder[3]) {
            toSwerveSpeeds = new ChassisSpeeds(variablesHolder[0], variablesHolder[1],  variablesHolder[2]);
            System.out.println(variablesHolder[0]);
            driveTrain.drive(toSwerveSpeeds);
        } else {
            toSwerveSpeeds = new ChassisSpeeds(0, 0, 0);
            driveTrain.drive(toSwerveSpeeds);
            System.out.println("Stop Auto");
        }


        // gives match time countdown not up, subtract from 15 to be accurate

    }

}
