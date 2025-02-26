
package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants.Levels;
import frc.robot.subsystems.Claw;

public class SetClawLevel extends Command {
  private Levels level;
  private Claw claw;

  public SetClawLevel(Levels level, Claw claw) {
    this.level = level;
    this.claw = claw;
    addRequirements(this.claw);
  }

  @Override
  public void execute() {
    claw.setShaftLevel(level);
  }
}
