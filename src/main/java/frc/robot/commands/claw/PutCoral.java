package frc.robot.commands.claw;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class PutCoral extends Command {
  private Claw claw;
  private int counter, condition_counter;
  private boolean start_count;
  private Supplier<Boolean> condition;

  public PutCoral(Claw claw, Supplier<Boolean> condition) {
    this.claw = claw;
    this.condition = condition;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    claw.stuckAlgae(false);
    counter = 0;
    condition_counter = 0;
    start_count = false;
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.PutPower);
    if(condition_counter < 6) condition_counter ++;
    if(!claw.detectCoral())start_count = true;
    if(start_count) counter ++;
  }

  @Override
  public void end(boolean interrupted) {
      claw.stuckAlgae(true);
      claw.setGrabberPower(0);
  }
    
  @Override
  public boolean isFinished() {
    if((counter >= ClawConstants.PutDelay / 0.05)) return true;
    else return false;
  }

}
