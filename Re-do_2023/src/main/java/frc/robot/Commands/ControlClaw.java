package frc.robot.Commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pneumatics;

public class ControlClaw extends CommandBase {
    // purely for theory, do not test n%=2 ifn=0 m1 if n=1 m2
    private final Value[] clawPos = { Value.kReverse, Value.kForward };
    private int n_Modifer;
    private final Pneumatics claw;

    public ControlClaw(Pneumatics clawSub) {
        claw = clawSub;
        addRequirements(clawSub);
    }

    @Override
    public void initialize() {
        n_Modifer = 1; // starts at 1, as the claw will start closed. When pressed the first time, it
                       // will open the claw. n_Mod will always be 1 behind expected
    }

    @Override
    public void execute() {
        n_Modifer %= 2;

        claw.setState(clawPos[n_Modifer]);

        // n_Modifer++; //this was here originally, but I think its makes more sense in
        // the isFinished for a "when pressed"
    }

    @Override
    public boolean isFinished() {
        if (claw.getState() == clawPos[n_Modifer]) {
            n_Modifer++;
            return true;
        } else {
            return false;
        }
    }

}
