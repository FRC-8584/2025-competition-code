package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberPower extends Command {
  private final Climber m_climber;
  private final Supplier<Double> m_setpoint;

  public SetClimberPower(Climber climber, Supplier<Double> setpoint) {
    m_climber = climber;
    m_setpoint = setpoint;
    addRequirements(m_climber);
  }

  @Override
  public void execute() {
    m_climber.setPower(m_setpoint.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.setPower(0);
  }

}
