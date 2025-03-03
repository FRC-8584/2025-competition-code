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
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.commands.PutCoralWithSwerve;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class RobotContainer {
  private Joystick js = new Joystick(0);
  
  private Swerve swerve = new Swerve();
  private Elevator elevator = new Elevator();
  private Claw claw  = new Claw();
  private Intake intake = new Intake();
  
  public RobotContainer() {
    swerve.setDefaultCommand(
      new ArcadeDrive(swerve, ()->-js.getY(), ()->-js.getX(), ()->-js.getRawAxis(4))
    );

    new RunCommand(
        ()->swerve.drive(
          OperationConstant.axieOptimizers[0].get(Tools.deadband(-js.getY(), 0.05)),
          OperationConstant.axieOptimizers[1].get(Tools.deadband(-js.getX(), 0.05)),
          OperationConstant.axieOptimizers[2].get(Tools.deadband(-js.getRawAxis(4), 0.05)), 
          false),
      swerve);

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {
    new JoystickButton(js, 3).and(()->LimelightHelpers.getTargetCount("limelight")!=0).onTrue(new MoveToReef(swerve, Reef.Left, 0.20));
    new JoystickButton(js, 2).and(()->LimelightHelpers.getTargetCount("limelight")!=0).onTrue(new MoveToReef(swerve, Reef.Right, 0.20));
  }

  private void configureLimelight(){
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 
      LimelightConstants.Z,    // Forward offset (meters)
      LimelightConstants.X,    // Side offset (meters)
      LimelightConstants.Y,   // Height offset (meters)
      LimelightConstants.Roll,    // Roll (degrees)
      LimelightConstants.Pitch,   // Pitch (degrees)
      LimelightConstants.Yaw   // Yaw (degrees)
    );
  }    

  private void configNamedCommands() {
    // NamedCommands.registerCommand("PutCoral", new PutCoral(swerve, elevator, claw, OperationConstant.Levels.L4));
    // NamedCommands.registerCommand("GetCoral", new GrabCoralTillGet(claw));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("red_test");
  }
}
