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
import frc.robot.commands.PutCoral;
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
    new JoystickButton(js, 1).onTrue(new PutCoral(elevator, claw, OperationConstant.Levels.L1));
    new JoystickButton(js, 2).onTrue(new PutCoral(elevator, claw, OperationConstant.Levels.L2));
    new JoystickButton(js, 3).onTrue(new PutCoral(elevator, claw, OperationConstant.Levels.L4));
    new JoystickButton(js, 4).onTrue(new PutCoral(elevator, claw, OperationConstant.Levels.L3));
  }

  private void configureLimelight(){
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 
    LimelightConstants.Y,    // Forward offset (meters)
    LimelightConstants.X,    // Side offset (meters)
    LimelightConstants.Height,    // Height offset (meters)
    LimelightConstants.Roll,    // Roll (degrees)
    LimelightConstants.Pitch,   // Pitch (degrees)
    LimelightConstants.Yaw   // Yaw (degrees)
);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
