// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperationConstant;
import frc.robot.Constants.OperationConstant.Keys;
import frc.robot.commands.GetCoral;
import frc.robot.commands.ToCoralLevel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class RobotContainer {
  private Joystick js = new Joystick(0);
  
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
    configureLimelight();
  }

  private void configureBindings() {
    new JoystickButton(js, Keys.GetCoral).onTrue(new GetCoral(claw, elevator, js));
    new JoystickButton(js, Keys.ToReefL1).onTrue(new ToCoralLevel(OperationConstant.Levels.L1, claw, elevator));
    new JoystickButton(js, Keys.ToReefL2).onTrue(new ToCoralLevel(OperationConstant.Levels.L2, claw, elevator));
    new JoystickButton(js, Keys.ToReefL3).onTrue(new ToCoralLevel(OperationConstant.Levels.L3, claw, elevator));
    new JoystickButton(js, Keys.ToReefL4).onTrue(new ToCoralLevel(OperationConstant.Levels.L4, claw, elevator));
  }

  private void configureLimelight(){
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 
    LimelightConstants.X,    // Forward offset (meters)
    LimelightConstants.Y,    // Side offset (meters)
    LimelightConstants.Height,    // Height offset (meters)
    0.0,    // Roll (degrees)
    0.0,   // Pitch (degrees)
    LimelightConstants.Angle    // Yaw (degrees)
);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
