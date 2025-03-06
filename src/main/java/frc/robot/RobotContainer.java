package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.ResetToAlgaeDefault;
import frc.robot.commands.ResetToDefault;
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

  private SendableChooser<Command> autoChooser;
  
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerve.setDefaultCommand(
      new ArcadeDrive(
        swerve,
        ()->-js1.getY(),
        ()->-js1.getX(),
        ()->-js1.getRawAxis(4),
        ()->js1.getRawButton(1),
        ()->js1.getRawButtonPressed(8)
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

    claw.setDefaultCommand(
      new ControlGrabber(claw, ()->js1.getRawAxis(5), ()->js1.getRawAxis(6))
    );

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
    new JoystickButton(js1, 7).onTrue(new ResetToDefault(claw, elevator, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    new JoystickButton(js2, 1).onTrue(new ToLevel(claw, elevator, Levels.Coral_L1));
    new JoystickButton(js2, 2).onTrue(new ToLevel(claw, elevator, Levels.Coral_L2));
    new JoystickButton(js2, 3).onTrue(new ToLevel(claw, elevator, Levels.Coral_L4));
    new JoystickButton(js2, 4).onTrue(new ToLevel(claw, elevator, Levels.Coral_L3));
    new JoystickButton(js2, 5).and(()->js2.getPOV() == 0).onTrue(new ToLevel(claw, elevator, Levels.Algea_L2));
    new JoystickButton(js2, 5).and(()->js2.getPOV() == 180).onTrue(new ToLevel(claw, elevator, Levels.Algea_L1));
    new JoystickButton(js2, 6).onTrue(new ToLevel(claw, elevator, Levels.DefaultWithAlgae));
    new JoystickButton(js2, 7).onTrue(new ResetToDefault(claw, elevator, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    new JoystickButton(js2, 8).onTrue(new ResetToAlgaeDefault(claw, elevator, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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
    NamedCommands.registerCommand("PutCoral",
      new SequentialCommandGroup(
        new ToLevel(claw, elevator, Levels.Coral_L4),
        new ToLevel(claw, elevator, Levels.Default)
      )
    );
    NamedCommands.registerCommand("GetCoral",
      new GrabCoral(claw)
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
