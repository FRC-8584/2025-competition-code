package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperationConstant.Keys;
import frc.robot.commands.claw.GrabCoral;
import frc.robot.commands.claw.SetClawLevel;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class GetCoral extends ParallelCommandGroup {
    public GetCoral(Claw claw, Elevator elevator, Joystick joystick) {
        addCommands(
            new ParallelCommandGroup(
                new SetClawLevel(ClawConstants.Levels.Default, claw),
                new SetElevatorHeight(elevator, ElevatorConstants.Levels.L1)
            ),
            new SetClawLevel(ClawConstants.Levels.L1, claw),
            new GrabCoral(claw, ()->joystick.getRawButton(Keys.StopGetCoral))
        );
    }
}
 