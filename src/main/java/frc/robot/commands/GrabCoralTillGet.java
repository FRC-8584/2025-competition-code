package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class GrabCoralTillGet extends Command {
  private Claw claw;
  private double counter;

  public GrabCoralTillGet(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    counter = 0;
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.GrabberPower);
    if(claw.isGet()) counter ++;
    System.out.println(counter);
  }

  @Override
  public void end(boolean interrupted) {
    claw.setGrabberPower(0);
  }

  @Override
  public boolean isFinished() {
    if((counter >= ClawConstants.SensorDelay / 0.05)) return true;
    else return false;
  }
}
