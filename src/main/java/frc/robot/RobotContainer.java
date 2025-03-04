package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.ToLevel;
import frc.robot.commands.claw.GrabAlgae;
import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.subsystems.*;
import frc.robot.Constants.Levels;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;
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

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {
    new JoystickButton(js, 1).onTrue(new ToLevel(claw, elevator, Levels.Algea_L1));
    new JoystickButton(js, 2).onTrue(new ToLevel(claw, elevator, Levels.Algea_L2));
    new JoystickButton(js, 4).onTrue(new ToLevel(claw, elevator, Levels.Default));
    new JoystickButton(js, 5).whileTrue(new GrabAlgae(claw));
  }

  private void configureLimelight(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
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
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("red_test");
  }
}
