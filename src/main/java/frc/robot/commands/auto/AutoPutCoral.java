// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Levels;
import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.PutCoral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPutCoral extends SequentialCommandGroup {
  /** Creates a new AutoPutCoral. */
  public AutoPutCoral(Claw claw, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToLevel(claw, elevator, Levels.Coral_L4)
    );
  }
}
