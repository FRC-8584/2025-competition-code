package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class SetGrabberPower extends Command {
  private double power;
  private Claw claw;
  public SetGrabberPower(double power, Claw claw) {
    this.power = power;
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void execute() {
    claw.setGrabberPower(power);
  }
    
  @Override
  public boolean isFinished() {
      return true;
  }

}
