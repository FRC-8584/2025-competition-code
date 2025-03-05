package frc.robot.commands.claw;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Tools;

public class ControlGrabber extends Command {
  private Claw claw;
  private Supplier<Double> a, b;
  private boolean stuck;

  public ControlGrabber (Claw claw, Supplier<Double> a, Supplier<Double> b) {
    this.claw = claw;
    this.a = a;
    this.b = b;
    addRequirements(this.claw);
  }

  @Override
  public void initialize() {
    claw.stuckAlgae(true);
    stuck = true;
  }

  @Override
  public void execute() {
    double power = a.get() - b.get();
    if(Tools.isInRange(power, -0.1, 0.1)) {
      if(stuck = false) {
        claw.setGrabberPower(0);
        stuck = true;
      }
    }
    else {
      stuck = false;
      claw.setGrabberPower(power);
    }
    claw.stuckAlgae(stuck);
  }

  @Override
  public void end(boolean interrupted) {
    claw.stuckAlgae(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
