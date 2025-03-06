// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Levels;
import frc.robot.commands.claw.PutCoral;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetToDefault extends SequentialCommandGroup {
  /** Creates a new ResetToDefault. */
  public ResetToDefault(
    Claw claw,
    Intake intake,
    Swerve swerve,
    Elevator elevator
  ) {
    addCommands(
      new StartEndCommand(()->swerve.drive(0, 0, 0,false), null, swerve),
      new StartEndCommand(()->{
        intake.setGrabberPower(-0.8);
        intake.setShaftPosition(0);
      }, null, intake),
      new PutCoral(claw),
      new ToLevel(claw, elevator, Levels.Default)
    );
  }
}
