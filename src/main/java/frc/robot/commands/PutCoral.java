package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperationConstant;
import frc.robot.commands.claw.SetGrabberPower;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class PutCoral extends SequentialCommandGroup {
    public PutCoral(Elevator elevator, Claw claw, OperationConstant.Levels level) {
        addCommands(
            new ToCoralLevel(elevator, claw, level),
            new SetGrabberPower(0.3, claw),
            new WaitCommand(0.3),
            new SetGrabberPower(0, claw)
        );
    }
}
