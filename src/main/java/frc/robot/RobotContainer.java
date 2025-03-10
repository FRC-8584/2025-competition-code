package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.ResetToAlgaeDefault;
import frc.robot.commands.ResetToDefault;
import frc.robot.commands.ToLevel;
import frc.robot.commands.auto.AutoPutCoral;
import frc.robot.commands.claw.ControlGrabber;
import frc.robot.commands.claw.GetCoral;
import frc.robot.commands.claw.PutCoral;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.commands.swerve.MoveToReef_V3;
import frc.robot.subsystems.*;
import frc.robot.Constants.Levels;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Reef;
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
        ()->js1.getRawAxis(2),
        ()->js1.getRawAxis(3))
    );
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

    claw.setDefaultCommand(
      new ControlGrabber(claw, ()->js1.getRawAxis(2), ()->js1.getRawAxis(3))
    );

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {
    new JoystickButton(js1, 2).whileTrue(new MoveToReef_V3(swerve, Reef.Right, ()->-js1.getY()));
    new JoystickButton(js1, 3).whileTrue(new MoveToReef_V3(swerve, Reef.Left, ()->-js1.getY()));
    new JoystickButton(js1, 4).whileTrue(new MoveToReef_V3(swerve, Reef.Medium, ()->-js1.getY()));
    new JoystickButton(js1, 5).and(()->!claw.detectCoral()).onTrue(new GetCoral(claw, ()->js1.getRawButtonPressed(5)));
    new JoystickButton(js1, 6).and(()->claw.detectCoral()).onTrue(new PutCoral(claw, ()->js1.getRawButtonPressed(6)));
    new JoystickButton(js1, 7).onTrue(new ResetToDefault(claw, elevator, intake, swerve).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    new JoystickButton(js2, 1).onTrue(new ToLevel(claw, elevator, Levels.Coral_L1));
    new JoystickButton(js2, 2).onTrue(new ToLevel(claw, elevator, Levels.Coral_L2));
    new JoystickButton(js2, 3).onTrue(new ToLevel(claw, elevator, Levels.Coral_L4));
    new JoystickButton(js2, 4).onTrue(new ToLevel(claw, elevator, Levels.Coral_L3));

    new JoystickButton(js2, 5).and(()->js2.getPOV() == 0).onTrue(new ToLevel(claw, elevator, Levels.Algea_L2));
    new JoystickButton(js2, 5).and(()->js2.getPOV() == 180).onTrue(new ToLevel(claw, elevator, Levels.Algea_L1));

    new JoystickButton(js2, 6).onTrue(new ToLevel(claw, elevator, Levels.DefaultWithAlgae));
    new JoystickButton(js2, 7).onTrue(new ResetToDefault(claw, elevator, intake, swerve).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    new JoystickButton(js2, 8).onTrue(new ResetToAlgaeDefault(swerve, claw, elevator, intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }

  private void configureLimelight(){
    // NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("pipeline").setNumber(
    //   DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? 1 : 0
    // );
    NetworkTableInstance.getDefault().getTable(LimelightConstants.device).getEntry("ledMode").setNumber(1);
  }

  private void configNamedCommands() {
    NamedCommands.registerCommand("PutCoral",
      new AutoPutCoral(claw, elevator)
    );
    NamedCommands.registerCommand("GetCoral",
      new GetCoral(claw, ()->{return false;})
    );
    NamedCommands.registerCommand("abcde",
      new PrintCommand("a")
    );
    NamedCommands.registerCommand("GrabAlgae_L1",
    new PrintCommand("a")
    );
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
