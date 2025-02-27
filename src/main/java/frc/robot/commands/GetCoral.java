package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.claw.GrabTillGet;
import frc.robot.commands.claw.SetClawLevel;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class GetCoral extends ParallelCommandGroup {
    public GetCoral(Claw claw, Elevator elevator) {
        addCommands(
            new ParallelCommandGroup(
                new SetClawLevel(ClawConstants.Levels.L2, claw),
                new SetElevatorHeight(elevator, ElevatorConstants.Levels.L1)
            ),
            new GrabTillGet(claw)
        );
    }
}
