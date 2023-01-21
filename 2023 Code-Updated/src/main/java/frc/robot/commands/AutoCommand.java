package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wpiVcontainer.FieldDriveSub;

public class AutoCommand extends CommandBase {
  private final FieldDriveSub m_subsystem;

  public AutoCommand(FieldDriveSub subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }


}
