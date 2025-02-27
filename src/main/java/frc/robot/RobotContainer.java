// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperationConstant;
import frc.robot.commands.claw.SetClawLevel;
import frc.robot.commands.claw.SetGrabberPower;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Tools;

public class RobotContainer {
  private Joystick js = new Joystick(0);
  private Joystick js2 = new Joystick(1);
  
  private Swerve swerve = new Swerve();
  private Elevator elevator = new Elevator();
  private Claw claw  = new Claw();
  
  public RobotContainer() {
    swerve.setDefaultCommand(
      new RunCommand(
        ()->swerve.drive(
          OperationConstant.axieOptimizers[0].get(Tools.deadband(-js.getY(), 0.05)),
          OperationConstant.axieOptimizers[1].get(Tools.deadband(-js.getX(), 0.05)),
          OperationConstant.axieOptimizers[2].get(Tools.deadband(-js.getRawAxis(4), 0.05)), 
          true),
        swerve)
    );

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(js, 1).onTrue(new SetElevatorHeight(elevator, ElevatorConstants.Levels.L1));
    new JoystickButton(js, 2).onTrue(new SetElevatorHeight(elevator, ElevatorConstants.Levels.L2));
    new JoystickButton(js, 3).onTrue(new SetElevatorHeight(elevator, ElevatorConstants.Levels.L3));
    new JoystickButton(js, 4).onTrue(new SetElevatorHeight(elevator, ElevatorConstants.Levels.L4));

    new JoystickButton(js2, 5).whileTrue(new SetGrabberPower(0.1, claw));
    new JoystickButton(js2, 5).whileTrue(new SetGrabberPower(-0.1, claw));

    new JoystickButton(js2, 1).onTrue(new SetClawLevel(ClawConstants.Levels.L1, claw));
    new JoystickButton(js2, 2).onTrue(new SetClawLevel(ClawConstants.Levels.Default, claw));
    new JoystickButton(js2, 3).onTrue(new SetClawLevel(ClawConstants.Levels.L2, claw));
    new JoystickButton(js2, 4).onTrue(new SetClawLevel(ClawConstants.Levels.L3, claw));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
