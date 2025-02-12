package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class RunGrapper extends Command {
  private final Claw m_claw;
  private final double m_setpoint;

  public RunGrapper(Claw claw, double setpoint) {
    m_claw = claw;
    m_setpoint = setpoint;
    addRequirements(m_claw);
  }

  @Override
  public void execute() {
    m_claw.setGrapperPower(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_claw.setGrapperPower(0);
  }

}
