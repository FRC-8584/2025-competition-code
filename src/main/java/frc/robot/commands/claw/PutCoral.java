package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class PutCoral extends Command {
  private Claw claw;
  private int counter;

  public PutCoral(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    counter = 0;
  }

  @Override
  public void execute() {
    claw.setGrabberPower(0.8);
    if(!claw.detectCoral()) counter++;
  }

  @Override
  public void end(boolean interrupted) {
      claw.setGrabberPower(0);
  }
    
  @Override
  public boolean isFinished() {
    if((counter >= ClawConstants.PutDelay / 0.05)) return true;
    else return false;
  }

}
