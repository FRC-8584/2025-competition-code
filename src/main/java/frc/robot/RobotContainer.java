package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.AimReefAlgae;
import frc.robot.commands.PutCoralWithSwerve;
import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.ClawGrabAlgae;
import frc.robot.commands.claw.ClawPutAlgae;
import frc.robot.commands.claw.GrabCoral;
import frc.robot.commands.intake.IntakeGrabAlgae;
import frc.robot.commands.intake.IntakePutAlgae;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.subsystems.*;
import frc.robot.Constants.Levels;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Reef;
import frc.robot.utils.LimelightHelpers;

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
        ()->-js1.getRawAxis(4)
      )
    );

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {
    new JoystickButton(js1, 1).onTrue   (new GrabCoral(claw)         
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js1, 2).whileTrue(new ClawGrabAlgae(claw)     
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js1, 3).whileTrue(new ClawPutAlgae(claw)      
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js1, 4).onTrue   (new ToLevel(claw, elevator, Levels.DefaultWithAlgae)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js1, 5).onTrue   (new IntakeGrabAlgae(intake) 
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js1, 6).onTrue   (new IntakePutAlgae(intake)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    new JoystickButton(js2, 1).and(()->js2.getPOV() == 90).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Right, Levels.Coral_L1));
    new JoystickButton(js2, 2).and(()->js2.getPOV() == 90).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Right, Levels.Coral_L2));
    new JoystickButton(js2, 3).and(()->js2.getPOV() == 90).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Right, Levels.Coral_L4));
    new JoystickButton(js2, 4).and(()->js2.getPOV() == 90).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Right, Levels.Coral_L3));
    new JoystickButton(js2, 1).and(()->js2.getPOV() == 270).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Left, Levels.Coral_L1));
    new JoystickButton(js2, 2).and(()->js2.getPOV() == 270).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Left, Levels.Coral_L2));
    new JoystickButton(js2, 3).and(()->js2.getPOV() == 270).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Left, Levels.Coral_L4));
    new JoystickButton(js2, 4).and(()->js2.getPOV() == 270).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new PutCoralWithSwerve(claw, elevator, swerve, Reef.Left, Levels.Coral_L3));
    new JoystickButton(js2, 6).and(()->js2.getPOV() == 0).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new AimReefAlgae(claw, elevator, swerve, Levels.Algea_L2));
    new JoystickButton(js2, 6).and(()->js2.getPOV() == 180).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).onTrue(new AimReefAlgae(claw, elevator, swerve, Levels.Algea_L1));
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
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("red_test");
  }
}
