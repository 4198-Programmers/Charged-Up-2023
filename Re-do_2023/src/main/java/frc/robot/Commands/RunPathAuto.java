package frc.robot.Commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.PathHolder;
import frc.robot.Subsystems.PathHolder.PathChoice;

public class RunPathAuto extends CommandBase {
    private final PathHolder path;
    private final DriveTrain driveTrain;
    private long timeStart;
    private double matchTime;
    private ChassisSpeeds toSwerveSpeeds;
    private double[] variablesHolder;
    HashMap<String, Command> map;
    List<EventMarker> markers;
    Command commands;
    List<EventMarker> unpassedMarkers;

    public RunPathAuto(PathHolder pathSub, DriveTrain driveArg) {
        this.path = pathSub;
        this.driveTrain = driveArg;
        addRequirements(driveArg);
    }

    @Override
    public void initialize() {
        timeStart = System.currentTimeMillis();
        map = path.getPathEventMap();
        markers = path.getPathMarks();
        unpassedMarkers = markers;
    }

    @Override
    public void execute() {
        matchTime = (double) ((System.currentTimeMillis() - timeStart) / 1000);
        variablesHolder = path.getPathVelocities(matchTime);

        for (PathPlannerTrajectory.EventMarker marker : markers) {
            for (String name : marker.names) {
                if (map.containsKey(name)) {
                    var reqs = map.get(name).getRequirements();
                    m_requirements.addAll(reqs);
                    // commands = map.get(name);
                    // commands.schedule();
                }
            }
        }

        if (unpassedMarkers.size() > 0 && matchTime >= unpassedMarkers.get(0).timeSeconds) {
            PathPlannerTrajectory.EventMarker marker = unpassedMarkers.remove(0);

            for (String name : marker.names) {
                if (map.containsKey(name)) {
                    commands = map.get(name);
                    commands.schedule();
                    System.out.println(matchTime + "time");
                } else {
                    System.out.println("cannot run commands?");
                }
            }

        } else {
            if (matchTime <= variablesHolder[3]) {
                toSwerveSpeeds = new ChassisSpeeds(variablesHolder[0], variablesHolder[1], variablesHolder[2]);
                driveTrain.drive(toSwerveSpeeds);
            } else {
                toSwerveSpeeds = new ChassisSpeeds(0, 0, 0);
                driveTrain.drive(toSwerveSpeeds);
                System.out.println("Stop Auto");
            }
        }

        // gives match time countdown not up, subtract from 15 to be accurate

    }

}
