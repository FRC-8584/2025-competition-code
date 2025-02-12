package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class JoystickSwerve extends Command{
  private Swerve m_swerve;
  private Supplier<Double> m_x, m_y, m_turn; 

  public JoystickSwerve(
    Swerve swerve,
    Supplier<Double> x,
    Supplier<Double> y,
    Supplier<Double> turn)
  {
    m_swerve = swerve;
    m_x = x;
    m_y = y;
    m_turn = turn;
    addRequirements(m_swerve);
  }
  
  @Override
  public void execute() {
    m_swerve.move(m_x.get(), -m_y.get(), m_turn.get());
  }
}
