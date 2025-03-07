// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Levels;
import frc.robot.commands.ToLevel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAlgae extends SequentialCommandGroup {
  /** Creates a new MoveBackward. */
  public GrabAlgae(
    Claw claw,
    Elevator elevator,
    Swerve swerve,
    Levels level
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ToLevel(claw, elevator, level),
        new StartEndCommand(()->swerve.drive(0.1, 0, 0, false), ()->{}, swerve),
        new StartEndCommand(()->{claw.setGrabberPower(0.8); claw.stuckAlgae(false);}, ()->{}, claw),
        new WaitCommand(0.5),
        new StartEndCommand(()->swerve.drive(-0.1, 0, 0, false), ()->{}, swerve),
        new StartEndCommand(()->{claw.setGrabberPower(0); claw.stuckAlgae(false);}, ()->{}, claw),
        new ToLevel(claw, elevator, Levels.DefaultWithAlgae)
    );
  }
}
