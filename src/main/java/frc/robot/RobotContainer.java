// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OperationConstant;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Tools;

public class RobotContainer {
  private Joystick js = new Joystick(0);
  
  private Swerve swerve = new Swerve();
  
  public RobotContainer() {
    swerve.setDefaultCommand(
      new RunCommand(
        ()->swerve.drive(
          OperationConstant.axieOptimizers[0].get(Tools.deadband(-js.getY(), 0.05)),
          OperationConstant.axieOptimizers[1].get(Tools.deadband(-js.getX(), 0.05)),
          OperationConstant.axieOptimizers[2].get(Tools.deadband(-js.getRawAxis(4), 0.05)), false),
        swerve)
    );

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
