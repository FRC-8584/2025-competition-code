package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperationConstant;
import frc.robot.Constants.OperationConstant.Levels;
import frc.robot.commands.claw.SetGrabberPower;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class PutCoral extends SequentialCommandGroup {
    public PutCoral(Swerve swerve, Elevator elevator, Claw claw, OperationConstant.Levels level) {
            addCommands(
                new ToCoralLevel(elevator, claw, level),
                new SetGrabberPower(0.5, claw),
                new WaitCommand(0.2),
                new SetGrabberPower(0, claw),
                new ToCoralLevel(elevator, claw, Levels.L1)
            );
    }
}
