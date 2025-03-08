package frc.robot.commands.claw;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.ClawConstants;

public class GetCoral extends Command {
  private Claw claw;
  private double counter, condition_counter;
  private Supplier<Boolean> condition;

  public GetCoral(Claw claw, Supplier<Boolean> condition) {
    this.claw = claw;
    this.condition = condition;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    claw.stuckAlgae(false);
    condition_counter = 0;
    counter = 0;
  }

  @Override
  public void execute() {
    claw.setGrabberPower(ClawConstants.GrabPower);
    if(claw.detectCoral()) counter ++;
    if(condition_counter < 6) condition_counter ++;
  }

  @Override
  public void end(boolean interrupted) {
    claw.setGrabberPower(0);
    claw.stuckAlgae(true);
  }

  @Override
  public boolean isFinished() {
    if((counter >= ClawConstants.GrabCoralDelay / 0.05) || (condition_counter >=6 && condition.get())) return true;
    else return false;
  }
}
