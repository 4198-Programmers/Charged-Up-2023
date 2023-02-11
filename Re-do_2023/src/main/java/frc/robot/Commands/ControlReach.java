package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ReachArmSub;

public class ControlReach extends CommandBase{
    private final ReachArmSub reach;
    private final DoubleSupplier speedSupplier;

    public ControlReach(ReachArmSub reachArg, DoubleSupplier supplier){
        reach = reachArg;
        speedSupplier = supplier;

        addRequirements(reachArg);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
            reach.moveReach(speedSupplier.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        reach.moveReach(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }




    
}
