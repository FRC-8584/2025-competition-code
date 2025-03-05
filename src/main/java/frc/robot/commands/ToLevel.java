package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Levels;
import frc.robot.commands.claw.SetClawLevel;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ToLevel extends SequentialCommandGroup {
    public ToLevel(Claw claw, Elevator elevator, Levels level) {
        Levels claw_level = level;
        Levels elevator_level = level;

        if(level == Levels.DefaultWithAlgae)
            addCommands(
                new SetClawLevel(level, claw),
                new SetElevatorLevel(elevator, elevator_level),
                new SetClawLevel(claw_level, claw)
            );
        
        else
            addCommands(
                new SetClawLevel(Levels.Dodge, claw),
                new SetElevatorLevel(elevator, elevator_level),
                new SetClawLevel(claw_level, claw)
            );
    }
}
