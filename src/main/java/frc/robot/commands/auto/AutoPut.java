package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Levels;
import frc.robot.Constants.Reef;
import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.PutCoral;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class AutoPut extends SequentialCommandGroup {
    public AutoPut(Swerve swerve, Elevator elevator, Claw claw, Reef reef, Levels levels) {
        addCommands(
            new MoveToReef(swerve, reef),
            new ToLevel(claw, elevator, levels),
            new PutCoral(claw)
        );
    }       
}
