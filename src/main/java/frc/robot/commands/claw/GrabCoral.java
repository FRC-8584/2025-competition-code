package frc.robot.commands.claw;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class GrabCoral extends Command {
  private Claw claw;
  private double counter;
  private Supplier<Boolean> stop;

  public GrabCoral(Claw claw, Supplier<Boolean> stop) {
    this.claw = claw;
    this.stop = stop;
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
    if((counter >= ClawConstants.SensorDelay / 0.05) || stop.get()) return true;
    else return false;
  }
}
