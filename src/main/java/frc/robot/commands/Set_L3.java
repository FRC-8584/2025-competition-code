package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class Set_L3 extends Command {
  private final Elevator m_Elevator;
  private final Claw m_claw;

  private final static double elevator_pos = 35;// cm
  private final static double claw_pos = 35;// deg

  public Set_L3(Elevator elevator, Claw claw) {
    m_Elevator = elevator;
    m_claw = claw;
    addRequirements(m_Elevator, m_claw);
  }

  @Override
  public void execute() {
    m_Elevator.setPosition(elevator_pos);
    m_claw.setShaftPosition(claw_pos);
  }

  @Override
  public void end(boolean interrupted) {}

}
