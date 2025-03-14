package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.Levels;
import frc.robot.utils.Tools;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorLevel extends Command {
  Elevator m_Elevator;
  Levels m_L = Levels.Default;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorLevel(Elevator subsystem, Levels level) {
    m_Elevator = subsystem;
    m_L = level;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.m_L = this.m_L;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Tools.isInRange(m_Elevator.getPosition(), m_L.getHeight() - 3.0, m_L.getHeight() + 3.0)) return true;
    else return false;
  }
}
