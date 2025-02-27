package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shaft;
import frc.robot.utils.Tools;
import frc.robot.Constants.ClawState;
import frc.robot.Constants.ShaftConstants;
import frc.robot.subsystems.Elevator;

public class SetClawState extends Command {
  private final Elevator m_Elevator;
  private final Shaft m_shaft;

  private ClawState m_state;

  private double shaftMinPos = ShaftConstants.kShaftMinPosition;
  private double shaftMaxPos = ShaftConstants.kShaftMaxPosition;

  public SetClawState(Elevator elevator, Shaft shaft, ClawState state) {
    m_Elevator = elevator;
    m_shaft = shaft;
    m_state = state;
    addRequirements(m_Elevator, m_shaft);
  }

  @Override
  public void execute() {
    double elevator_setpoint = m_state.getElevatorValue();
    double shaft_setpoint = m_state.getClawValue();

    double present_elevator_pos = m_Elevator.getPosition();

    if(Tools.isInRange(present_elevator_pos, 5, 50))
      shaftMinPos = 20;
    else
      shaftMinPos = ShaftConstants.kShaftMinPosition;

    if(Tools.isInRange(present_elevator_pos, 10, 13))
      shaftMaxPos = 170;
    else if(Tools.isInRange(present_elevator_pos, 7, 10))
      shaftMaxPos = 140;
    else if(Tools.isInRange(present_elevator_pos, 0, 7))
      shaftMaxPos = 120;
    else
      shaftMaxPos = ShaftConstants.kShaftMaxPosition;

    shaft_setpoint = Tools.bounding(shaft_setpoint, shaftMinPos, shaftMaxPos);

    m_Elevator.setPosition(elevator_setpoint);
    m_shaft.setPosition(shaft_setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

}
