package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shaft;
import frc.robot.utils.Tools;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class SetClawState extends Command {
  private final Elevator m_Elevator;
  private final Shaft m_shaft;

  private double m_elevator_pos = 0;// cm
  private double m_shaft_pos = 0;// deg

  private double shaftMinPos = Constants.ShaftConstants.kShaftMinPosition;
  private double shaftMaxPos = Constants.ShaftConstants.kShaftMaxPosition;

  public SetClawState(Elevator elevator, Shaft shaft, double elevator_pos, double shaft_pos) {
    m_Elevator = elevator;
    m_shaft = shaft;
    m_elevator_pos = elevator_pos;
    m_shaft_pos = shaft_pos;
    addRequirements(m_Elevator, m_shaft);
  }

  @Override
  public void execute() {
    double elevator_setpoint = m_elevator_pos;
    double shaft_setpoint = m_shaft_pos;

    double present_elevator_pos = m_Elevator.getPosition();

    if(Tools.isInRange(present_elevator_pos, 15, 50)){
      shaftMinPos = 25;
    }
    else{
      shaftMinPos = 0;
    }
    shaft_setpoint = Tools.bounding(shaft_setpoint, shaftMinPos, shaftMaxPos);

    m_Elevator.setPosition(elevator_setpoint);
    m_shaft.setPosition(shaft_setpoint);
  }

  @Override
  public void end(boolean interrupted) {}

}
