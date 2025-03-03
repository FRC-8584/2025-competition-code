package frc.robot.commands.shaft;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shaft;

public class SetShaftPosition extends Command {
  private final Shaft m_shaft;
  private final double m_setpoint;

  public SetShaftPosition(Shaft shaft, double setpoint) {
    m_shaft = shaft;
    m_setpoint = setpoint;
    addRequirements(m_shaft);
  }

  @Override
  public void execute() {
    m_shaft.setPosition(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

}
