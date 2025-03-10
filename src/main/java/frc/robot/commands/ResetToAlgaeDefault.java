// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Levels;
import frc.robot.commands.claw.SetClawLevel;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetToAlgaeDefault extends SequentialCommandGroup {
  /** Creates a new ResetToDefault. */
  public ResetToAlgaeDefault(
    Claw claw,
    Elevator elevator,
    Intake intake
  ) {
    addCommands(
      new SetClawLevel(Levels.DefaultWithAlgae, claw),
      new SetElevatorLevel(elevator, Levels.Default)
    );
  }
}
