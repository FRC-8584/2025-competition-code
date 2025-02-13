package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shaft;
import frc.robot.subsystems.Elevator;

public class Set_L1 extends Command {
  private final Elevator m_Elevator;
  private final Shaft m_shaft;

  private final static double elevator_pos = 0;// cm
  private final static double shaft_pos = 0;// deg

  public Set_L1(Elevator elevator, Shaft shaft) {
    m_Elevator = elevator;
    m_shaft = shaft;
    addRequirements(m_Elevator, m_shaft);
  }

  @Override
  public void execute() {
    m_Elevator.setPosition(elevator_pos);
    m_shaft.setPosition(shaft_pos);
  }

  @Override
  public void end(boolean interrupted) {}

}
