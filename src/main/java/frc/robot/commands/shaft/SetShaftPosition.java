package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shaft;

public class SetShaftPosition extends Command {
  private final Shaft m_claw;
  private final double m_setpoint;

  public SetShaftPosition(Shaft claw, double setpoint) {
    m_claw = claw;
    m_setpoint = setpoint;
    addRequirements(m_claw);
  }

  @Override
  public void execute() {
    m_claw.setPosition(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

}
