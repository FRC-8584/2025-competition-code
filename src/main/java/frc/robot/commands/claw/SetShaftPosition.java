package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class SetShaftPosition extends Command {
  private final Claw m_claw;
  private final double m_setpoint;

  public SetShaftPosition(Claw claw, double setpoint) {
    m_claw = claw;
    m_setpoint = setpoint;
    addRequirements(m_claw);
  }

  @Override
  public void execute() {
    m_claw.setShaftPosition(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

}
