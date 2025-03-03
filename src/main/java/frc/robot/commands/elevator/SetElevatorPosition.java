package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends Command {
  private final Elevator m_elevator;
  private final double m_setpoint;

  public SetElevatorPosition(Elevator elevator, double setpoint) {
    m_elevator = elevator;
    m_setpoint = setpoint;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.setPosition(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

}
