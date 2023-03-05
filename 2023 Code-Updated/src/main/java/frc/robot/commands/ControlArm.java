package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VertArm;

public class ControlArm extends CommandBase {
    VertArm vertArm;
    Joystick joystick;

    public ControlArm(VertArm vertArmArg, Joystick joystickRight) {
        vertArm = vertArmArg;
        joystick = joystickRight;
        addRequirements(vertArm);
    }

    @Override
    public void execute() {
        vertArm.moveArmUp(joystick.getY());
    }

}
