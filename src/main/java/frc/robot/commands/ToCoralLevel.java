package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperationConstant;
import frc.robot.commands.claw.SetClawLevel;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ToCoralLevel extends SequentialCommandGroup {
    public ToCoralLevel(Elevator elevator, Claw claw, OperationConstant.CoralLevels level) {
        ClawConstants.Levels claw_level = ClawConstants.Levels.L1;
        ElevatorConstants.Levels elevator_level =ElevatorConstants.Levels.L1;
        switch (level) {
            case Coral_L1:
                elevator_level = ElevatorConstants.Levels.L1;
                claw_level = ClawConstants.Levels.L1;
                break;
            case Coral_L2:
                elevator_level = ElevatorConstants.Levels.L2;
                claw_level = ClawConstants.Levels.L2;
                break;
            case Coral_L3:
                elevator_level = ElevatorConstants.Levels.L3;
                claw_level = ClawConstants.Levels.L2;
                break;
            case Coral_L4:
                elevator_level = ElevatorConstants.Levels.L4;
                claw_level = ClawConstants.Levels.L3;
                break;
            case Algea_L1:
                elevator_level = ElevatorConstants.Levels.L2;
                claw_level = ClawConstants.Levels.L4;
                break;
            case Algea_L2:
                elevator_level = ElevatorConstants.Levels.L3;
                claw_level = ClawConstants.Levels.L4;
                break;
            default:
                break;
        }
        addCommands(
            new ParallelCommandGroup(
                new SetClawLevel(ClawConstants.Levels.Default, claw),
                new SetElevatorHeight(elevator, elevator_level)
            ),
            new SetClawLevel(claw_level, claw)
        );
    }
}
