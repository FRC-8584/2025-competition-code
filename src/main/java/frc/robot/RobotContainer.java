package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.swerve.ArcadeDrive;
import frc.robot.subsystems.*;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperationConstant;

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
      () -> swerve.drive(
        OperationConstant.axieOptimizers[0].get(Tools.deadband(-js.getY(), 0.05)),
        OperationConstant.axieOptimizers[1].get(Tools.deadband(-js.getX(), 0.05)),
        OperationConstant.axieOptimizers[2].get(Tools.deadband(-js.getRawAxis(4), 0.05)), 
        false
      ), swerve
    );

    configureBindings();
    configureLimelight();
    configNamedCommands();
  }

  private void configureBindings() {

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
