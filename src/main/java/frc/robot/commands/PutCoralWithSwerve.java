package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperationConstant;
import frc.robot.Constants.OperationConstant.Levels;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.commands.claw.SetGrabberPower;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class PutCoralWithSwerve extends SequentialCommandGroup {
    public PutCoralWithSwerve(Swerve swerve, Elevator elevator, Claw claw, OperationConstant.Levels level, Reef reef) {
            addCommands(
                new ParallelCommandGroup(
                    new ToCoralLevel(elevator, claw, level),
                    new MoveToReef(swerve, reef)
                ),
                new SetGrabberPower(0.5, claw),
                new WaitCommand(0.2),
                new SetGrabberPower(0, claw),
                new ToCoralLevel(elevator, claw, Levels.L1)
            );
    }
}
