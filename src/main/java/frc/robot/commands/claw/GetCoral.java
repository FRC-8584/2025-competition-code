package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class GetCoral extends Command {
  private Claw claw;
  private double counter;

  public GetCoral(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    claw.stuckAlgae(false);
    counter = 0;
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.GrabPower);
    if(claw.detectCoral()) counter ++;
  }

  @Override
  public void end(boolean interrupted) {
    claw.setGrabberPower(0);
    claw.stuckAlgae(true);
  }

  @Override
  public boolean isFinished() {
    if((counter >= ClawConstants.GrabCoralDelay / 0.05)) return true;
    else return false;
  }
}
