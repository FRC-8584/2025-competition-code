package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.grabber.*;

import frc.robot.Constants.Reef;
import frc.robot.Constants.ClawState;
import frc.robot.Constants.LimelightConstants;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Shaft shaft = new Shaft();
  private final Grabber grabber = new Grabber();

  private final Joystick joystick_1 = new Joystick(Constants.OperatorConstants.Player1Port);
  // private final Joystick joystick_2 = new Joystick(Constants.OperatorConstants.Player2Port);

  public RobotContainer() {
    player1CommandList();
    // player2CommandList();
    configureLimelight();
  }

  private void configureLimelight(){
    LimelightHelpers.setCameraPose_RobotSpace(
      LimelightConstants.device, 
      LimelightConstants.X,    // Forward offset (meters)
      LimelightConstants.Z,    // Side offset (meters)
      LimelightConstants.Y,   // Height offset (meters)
      LimelightConstants.Roll,    // Roll (degrees)
      LimelightConstants.Pitch,   // Pitch (degrees)
      LimelightConstants.Yaw    // Yaw (degrees)
    );
  }

  private void player1CommandList() {
    final Joystick js = joystick_1;

    // swerve
    swerve.setDefaultCommand(
      new RunCommand(
        () ->
        swerve.drive(
          -Constants.OperatorConstants.axieOptimizers[0].get(Tools.deadband(js.getY(), 0.05)),
          -Constants.OperatorConstants.axieOptimizers[1].get(Tools.deadband(js.getX(), 0.05)),
          -Constants.OperatorConstants.axieOptimizers[2].get(Tools.deadband(js.getRawAxis(4), 0.05)),
          false
        ), swerve
      )
    );

    new JoystickButton(js, 7).onTrue(new AimReef(swerve, Reef.Left).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 8).onTrue(new AimReef(swerve, Reef.Right).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Claw Grapper
    new JoystickButton(js, 5)// Grapper Forward, Button LB
      .whileTrue(new SetGrabberPower(grabber, 1)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
    new JoystickButton(js, 6)// Grapper Reverse, Button RB
      .whileTrue(new SetGrabberPower(grabber, -1)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    // ClawState Combinate command CORAL Reef & Source
    clawStateBinding(js, 1, ClawState.DEFAULT_STATE);// Claw default state(L1), Button A
    clawStateBinding(js, 2, ClawState.PUT_CORAL_L2);// Claw L2, Button B
    clawStateBinding(js, 4, ClawState.PUT_CORAL_L3);// Claw L3, Button Y
    clawStateBinding(js, 3, ClawState.PUT_CORAL_L4);// Claw L4, Button X

    // ClawState Combinate command ALGAE Reef & Processor
    // clawStateBinding(js, 7, ClawState.GET_ALGAE_HIGH);
    // clawStateBinding(js, 8, ClawState.PUT_ALGAE_PROCESSOR);
  }

  // private void player2CommandList() {
  //   final Joystick js = joystick_2;
  // }

  private void clawStateBinding(Joystick js, int button_number, ClawState state) {
    new JoystickButton(js, button_number)
      .onTrue(new SetClawState(elevator, shaft, state)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }
}
