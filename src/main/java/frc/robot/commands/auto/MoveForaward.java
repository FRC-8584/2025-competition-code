package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class MoveForaward extends Command {
  private double seconds, counter;
  private Swerve swerve;

  public MoveForaward(Swerve swerve, double seconds) {
    this.seconds = seconds;
    this.swerve = swerve;
    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    counter = 0;
    swerve.drive(0.20, 0, 0, false);
  }

  @Override
  public void execute() {
    counter++;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    if(counter > seconds / 0.05) return true;
    else return false;
  }
}
