package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grabber;

public class SetGrabberPower extends Command {
  private final Grabber m_grabber;
  private final double m_setpoint;

  public SetGrabberPower(Grabber grabber, double setpoint) {
    m_grabber = grabber;
    m_setpoint = setpoint;
    addRequirements(m_grabber);
  }

  @Override
  public void execute() {
    m_grabber.setPower(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_grabber.setPower(0);
  }

}
