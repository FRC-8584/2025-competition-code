package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Levels;
import frc.robot.Constants.Reef;
import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.PutCoral;
import frc.robot.commands.swerve.MoveForaward;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class AutoPutCoral extends SequentialCommandGroup {
    public AutoPutCoral(Swerve swerve, Elevator elevator, Claw claw, Reef reef, Levels levels) {
        if(levels == Levels.Coral_L4) {
            addCommands(
                new ParallelCommandGroup(
                    new MoveToReef(swerve, reef),
                    new ToLevel(claw, elevator, levels)
                ),
                new MoveForaward(swerve, 0.2, 1.0),
                new PutCoral(claw)
            );
        }
        else {
            addCommands(
                new MoveToReef(swerve, reef),
                new ToLevel(claw, elevator, levels),
                new WaitCommand(0.3),
                new PutCoral(claw)
            );
        }
    }       
}
