package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class GrabTillGet extends Command {
  private Claw claw;
  private double counter;

  public GrabTillGet(Claw claw) {
    this.claw = claw;
    counter = 0;
    addRequirements(this.claw);
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.GrabberPower);
    if(claw.isGet()) counter ++;
  }

  @Override
  public void end(boolean interrupted) {
    claw.setGrabberPower(0);
  }

  @Override
  public boolean isFinished() {
    if(counter >= ClawConstants.SensorDelay / 0.05) return true;
    else return false;
  }
}
