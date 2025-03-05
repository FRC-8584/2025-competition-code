package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class PutAlgae extends Command {
  private Claw claw;

  public PutAlgae(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    claw.stuckAlgae(false);
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.PutPower);
  }

  @Override
  public void end(boolean interrupted) {
    claw.setGrabberPower(0);
    claw.stuckAlgae(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
