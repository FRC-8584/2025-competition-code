package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class PutCoral extends Command {
  private Claw claw;
  private int counter;
  private boolean start_count;

  public PutCoral(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    claw.stuckAlgae(false);
    counter = 0;
    start_count = false;
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.PutPower);
    if(!claw.detectCoral()) counter++;
  }

  @Override
  public void end(boolean interrupted) {
      claw.stuckAlgae(true);
      claw.setGrabberPower(0);
  }
    
  @Override
  public boolean isFinished() {
    if(!claw.detectCoral())start_count = true;
    if(start_count) counter ++;
    if((counter >= ClawConstants.PutDelay / 0.05)) return true;
    else return false;
  }

}
