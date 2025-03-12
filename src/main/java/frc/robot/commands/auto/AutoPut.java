package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        if(levels == Levels.Coral_L4) {
            addCommands(
                new ParallelCommandGroup(
                    new MoveToReef(swerve, reef),
                    new ToLevel(claw, elevator, levels)
                ),
                new MoveForaward(swerve, 1.0),
                new PutCoral(claw)
            );
        }
        else {
            addCommands(
                new MoveToReef(swerve, reef),
                new ToLevel(claw, elevator, levels),
                new PutCoral(claw)
            );
        }
    }       
}
