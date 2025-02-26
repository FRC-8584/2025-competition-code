
package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class SetClawLevel extends Command {
  private int level;
  private Claw claw;

  public SetClawLevel(int level, Claw claw) {
    this.level = level;
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void execute() {
    claw.setShaftLevel(level);
  }
}
