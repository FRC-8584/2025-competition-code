package frc.robot.commands.shaft;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shaft;

public class DefaultShaft extends Command {
  private final Shaft m_shaft;

  public DefaultShaft(Shaft claw) {
    m_shaft = claw;
    addRequirements(m_shaft);
  }

  @Override
  public void execute() {
    m_shaft.setPosition(m_shaft.getPosition());
  }

  @Override
  public void end(boolean interrupted) {}

}
