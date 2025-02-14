package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DefaultElevator extends Command {
  private final Elevator m_elevator;

  public DefaultElevator(Elevator elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.setPosition(m_elevator.getPosition());
  }

  @Override
  public void end(boolean interrupted) {}

}
