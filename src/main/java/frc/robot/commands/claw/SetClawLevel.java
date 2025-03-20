package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.Levels;
import frc.robot.utils.Tools;

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

  @Override
  public boolean isFinished() {
    if(Tools.isInRange(claw.getPosition(), level.getAngle() - 1.0, level.getAngle() + 1.0)) return true;
    else return false;
  }
}
