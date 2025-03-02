package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

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
    if(!claw.isGet()) counter++;
    System.out.println(counter);
  }
    
  @Override
  public boolean isFinished() {
    if(counter>50) return true;
    else return false;
  }

}
