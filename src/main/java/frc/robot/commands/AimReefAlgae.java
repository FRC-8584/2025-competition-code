// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Levels;
import frc.robot.Constants.Reef;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimReefAlgae extends ParallelCommandGroup {
  /** Creates a new AimReefAlgae. */
  public AimReefAlgae(Claw claw, Elevator elevator, Swerve swerve, Levels level) {
    addCommands(
      new MoveToReef(swerve, Reef.Medium),
      new ToLevel(claw, elevator, level)
    );
  }
}
