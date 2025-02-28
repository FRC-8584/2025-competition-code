package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OperationConstant.Levels;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class PutCoralWithSwerve extends SequentialCommandGroup {
    public PutCoralWithSwerve(
        Levels level,
        Joystick joystick,
        Swerve swerve,
        Claw claw,
        Elevator elevator
    )
    {
        addCommands(
            new ParallelCommandGroup(
                new ToCoralLevel(level, claw, elevator),
                new WaitUntilCommand(()->joystick.getPOV() == 90 || joystick.getPOV() == -90)
            ),
            new SwerveAimReef(swerve, (joystick.getPOV() == 90 || joystick.getPOV() == -90) ? Reef.Right : Reef.Left)
        );
    }
}
