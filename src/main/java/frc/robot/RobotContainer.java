package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.ControlGrabber;
import frc.robot.commands.claw.GrabCoral;
import frc.robot.commands.claw.PutCoral;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.*;
import frc.robot.Constants.Levels;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Reef;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class RobotContainer {
  private Joystick js1 = new Joystick(0);
  private Joystick js2 = new Joystick(1);
  
  private Swerve swerve = new Swerve();
  private Claw claw = new Claw();
  private Intake intake = new Intake();
  private Elevator elevator = new Elevator();
  
  public RobotContainer() {
    swerve.setDefaultCommand(
      new ArcadeDrive(
        swerve,
        ()->-js1.getY(),
        ()->-js1.getX(),
        ()->-js1.getRawAxis(4),
        ()->js1.getRawButton(1)
      )
    );
    claw.setDefaultCommand(
      new ControlGrabber(
        claw,
        ()->js1.getRawAxis(5),
        ()->js1.getRawAxis(6))
    );
    intake.setDefaultCommand(
      new RunCommand(
        ()->{
          double power = Tools.deadband(js2.getRawAxis(5), 0.2);
          if(power != 0) {
            intake.setGrabberPower(power);
            intake.setShaftPosition(53);
          }
          else{
            intake.setGrabberPower(0);
            intake.setShaftPosition(20);
          }
        },
        intake));

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {
    new JoystickButton(js1, 2).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new MoveToReef(swerve,Reef.Right));
    new JoystickButton(js1, 3).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new MoveToReef(swerve,Reef.Left));
    new JoystickButton(js1, 4).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new MoveToReef(swerve,Reef.Medium));
    new JoystickButton(js1, 5).and(()->!claw.detectCoral()).whileTrue(new GrabCoral(claw));
    new JoystickButton(js1, 6).and(()->claw.detectCoral()).onTrue(new PutCoral(claw));

    new JoystickButton(js2, 1).onTrue(new ToLevel(claw, elevator, Levels.Coral_L1));
    new JoystickButton(js2, 2).onTrue(new ToLevel(claw, elevator, Levels.Coral_L2));
    new JoystickButton(js2, 3).onTrue(new ToLevel(claw, elevator, Levels.Coral_L4));
    new JoystickButton(js2, 4).onTrue(new ToLevel(claw, elevator, Levels.Coral_L3));
    new JoystickButton(js2, 5).and(()->js2.getPOV() == 0).onTrue(new ToLevel(claw, elevator, Levels.Algea_L2));
    new JoystickButton(js2, 5).and(()->js2.getPOV() == 180).onTrue(new ToLevel(claw, elevator, Levels.Algea_L1));
    new JoystickButton(js2, 6).onTrue(new ToLevel(claw, elevator, Levels.DefaultWithAlgae));
  }


  private void configureLimelight(){
    LimelightHelpers.setCameraPose_RobotSpace(
      LimelightConstants.device,  // Device name
      LimelightConstants.Z,       // Forward offset (meters)
      LimelightConstants.X,       // Side offset (meters)
      LimelightConstants.Y,       // Height offset (meters)
      LimelightConstants.Roll,    // Roll (degrees)
      LimelightConstants.Pitch,   // Pitch (degrees)
      LimelightConstants.Yaw      // Yaw (degrees)
    );
  }

  private void configNamedCommands() {
    NamedCommands.registerCommand("PutCoral", null);
    NamedCommands.registerCommand("GetCoral", null);
    NamedCommands.registerCommand("GrabAlgae_L1", null);
    NamedCommands.registerCommand("GrabAlgae_L2", null);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("red_test");
  }
}
