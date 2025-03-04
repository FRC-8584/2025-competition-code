package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.commands.swerve.MoveToReef;
import frc.robot.subsystems.*;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperationConstant;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class RobotContainer {
  private Joystick js = new Joystick(0);
  
  private Swerve swerve = new Swerve();
  
  public RobotContainer() {
    swerve.setDefaultCommand(
      new ArcadeDrive(
        swerve,
        ()->-js.getY(),
        ()->-js.getX(),
        ()->-js.getRawAxis(4)
      )
    );

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {
    new JoystickButton(js, 3)
      .and(() -> LimelightHelpers.getTargetCount(LimelightConstants.device) != 0)
      .onTrue(new MoveToReef(swerve, Reef.Left)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    new JoystickButton(js, 2)
      .and(() -> LimelightHelpers.getTargetCount(LimelightConstants.device) != 0)
      .onTrue(new MoveToReef(swerve, Reef.Right)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
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
