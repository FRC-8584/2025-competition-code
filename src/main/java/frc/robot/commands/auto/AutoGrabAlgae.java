package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.Levels;
import frc.robot.Constants.Reef;
import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.ControlGrabber;
import frc.robot.commands.swerve.MoveForaward;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class AutoGrabAlgae extends SequentialCommandGroup {
  private double zero = 0;
  public AutoGrabAlgae(Claw claw, Elevator elevator, Swerve swerve, Levels level) {
    addCommands(
      new ToLevel(claw, elevator, level),
      new ParallelDeadlineGroup(
        new MoveToReef(swerve, Reef.Medium),
        new ControlGrabber(claw, ()->{return -ClawConstants.GrabPower;}, ()->{return zero;})
      ),
      new MoveForaward(swerve, -0.5, 1.0)
    );
  }
}
