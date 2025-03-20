package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ToLevel;
import frc.robot.commands.auto.AutoGrabAlgae;
import frc.robot.commands.auto.AutoPutCoral;
import frc.robot.commands.claw.ControlGrabber;
import frc.robot.commands.claw.GetCoral;
import frc.robot.commands.claw.PutCoral;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.*;
import frc.robot.Constants.Levels;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Reef;
import frc.robot.utils.LimelightHelpers;

public class RobotContainer {
  private Joystick js1 = new Joystick(0);
  private Joystick js2 = new Joystick(1);
  
  private Swerve swerve;
  private Claw claw;
  // private Intake intake;
  private Elevator elevator;

  private SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    createInstances();
    configDefaults();
    configBindings();
    configLimelight();
    configNamedCommands();
  }

  private void createInstances() {
    swerve = new Swerve();
    claw = new Claw();
    elevator = new Elevator();
  }

  private void configDefaults() {
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
        ()->js1.getRawAxis(2),
        ()->js1.getRawAxis(3))
    );
    /*  Deleted
    intake.setDefaultCommand(
      new RunCommand(
        ()->{
          double power = -Tools.deadband(js2.getRawAxis(2) - js2.getRawAxis(3), 0.2);
          if(power < 0) {
            intake.setGrabberPower(power * 0.5);
            intake.setShaftPosition(53);
          }
          else if(power > 0) {
            intake.setGrabberPower(power * 0.5);
            intake.setShaftPosition(20);
          }
          else{
            intake.setGrabberPower(0);
            intake.setShaftPosition(0);
          }
        },
        intake));
    */

    claw.setDefaultCommand(
      new ControlGrabber(claw, ()->js1.getRawAxis(2), ()->js1.getRawAxis(3))
    );
  }

  private void configBindings() {
    new JoystickButton(js1, 2).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).whileTrue(new MoveToReef(swerve, Reef.Right));
    new JoystickButton(js1, 3).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).whileTrue(new MoveToReef(swerve, Reef.Left));
    new JoystickButton(js1, 4).and(()->LimelightHelpers.getTargetCount(LimelightConstants.device)!=0).whileTrue(new MoveToReef(swerve, Reef.Medium));
    new JoystickButton(js1, 5).and(()->!claw.detectCoral()).onTrue(new GetCoral(claw));
    new JoystickButton(js1, 6).and(()->claw.detectCoral()).onTrue(new PutCoral(claw));

    new JoystickButton(js2, 1).onTrue(new ToLevel(claw, elevator, Levels.Coral_L1));
    new JoystickButton(js2, 2).onTrue(new ToLevel(claw, elevator, Levels.Coral_L2));
    new JoystickButton(js2, 3).onTrue(new ToLevel(claw, elevator, Levels.Coral_L4));
    new JoystickButton(js2, 4).onTrue(new ToLevel(claw, elevator, Levels.Coral_L3));

    new JoystickButton(js2, 5).and(()->js2.getPOV() == 0).onTrue(new ToLevel(claw, elevator, Levels.Algea_L2));
    new JoystickButton(js2, 5).and(()->js2.getPOV() == 180).onTrue(new ToLevel(claw, elevator, Levels.Algea_L1));
  
    new JoystickButton(js2, 6).onTrue(new ToLevel(claw, elevator, Levels.DefaultWithAlgae));
    new JoystickButton(js2, 7).onTrue(new ToLevel(claw, elevator, Levels.Net));
  }

  private void configLimelight(){
    NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(3);
  }

  private void configNamedCommands() {
    NamedCommands.registerCommand(
      "PutCoral_Right_L4",
      new AutoPutCoral(
        swerve, 
        elevator, 
        claw, 
        Reef.Right, 
        Levels.Coral_L4
      )
    );
    NamedCommands.registerCommand(
      "PutCoral_Left_L4",
      new AutoPutCoral(
        swerve, 
        elevator, 
        claw, 
        Reef.Left, 
        Levels.Coral_L4
      )
    );
    NamedCommands.registerCommand(
      "GetCoral",
      new GetCoral(claw)
    );
    NamedCommands.registerCommand(
      "GetAlgae_L1",
      new AutoGrabAlgae(
        claw,
        elevator, 
        swerve, 
        Levels.Algea_L1));
      NamedCommands.registerCommand(
        "GetAlgae_L2",
        new AutoGrabAlgae(
          claw,
          elevator, 
          swerve, 
          Levels.Algea_L2));  
    autoChooser =  AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}