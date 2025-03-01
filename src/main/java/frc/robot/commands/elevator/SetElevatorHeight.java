// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Levels;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Tools;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeight extends Command {
  Elevator m_Elevator;
  Levels m_L = Levels.L1;
  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(Elevator subsystem, Levels level) {
    m_Elevator = subsystem;
    m_L = level;
    addRequirements(m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.m_L = this.m_L;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(this.m_L) {
      case L1:
        if(Tools.isInRange(ElevatorConstants.Level_1_Height, ElevatorConstants.Level_1_Height-1.0, ElevatorConstants.Level_1_Height+1.0)){
          return true;
        }
      case L2:
        if(Tools.isInRange(ElevatorConstants.Level_2_Height, ElevatorConstants.Level_2_Height-1.0, ElevatorConstants.Level_2_Height+1.0)){
          return true;
        }
      case L3:
        if(Tools.isInRange(ElevatorConstants.Level_3_Height, ElevatorConstants.Level_3_Height-1.0, ElevatorConstants.Level_3_Height+1.0)){
          return true;
        }
      case L4:
        if(Tools.isInRange(ElevatorConstants.Level_4_Height, ElevatorConstants.Level_4_Height-1.0, ElevatorConstants.Level_4_Height+1.0)){
          return true;
        }
      default:
        break;
    }
    return false;
  }
}
