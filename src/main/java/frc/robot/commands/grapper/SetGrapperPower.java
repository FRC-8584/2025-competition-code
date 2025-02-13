package frc.robot.commands.grapper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grapper;

public class SetGrapperPower extends Command {
  private final Grapper m_grapper;
  private final double m_setpoint;

  public SetGrapperPower(Grapper grapper, double setpoint) {
    m_grapper = grapper;
    m_setpoint = setpoint;
    addRequirements(m_grapper);
  }

  @Override
  public void execute() {
    m_grapper.setPower(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_grapper.setPower(0);
  }

}
