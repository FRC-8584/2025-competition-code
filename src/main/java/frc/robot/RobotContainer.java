package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.grabber.*;

import frc.robot.Constants.ClawState;
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

    // Claw Grapper
    new JoystickButton(js, 5)
      .whileTrue(new SetGrabberPower(grabber, 1)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
    new JoystickButton(js, 6)
      .whileTrue(new SetGrabberPower(grabber, -1)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    // ClawState Combinate command CORAL Reef & Source
    setButtonToClawState(js, 1, ClawState.PUT_CORAL_REEF_L1);
    setButtonToClawState(js, 2, ClawState.PUT_CORAL_REEF_L2);
    setButtonToClawState(js, 4, ClawState.PUT_CORAL_REEF_L3);
    setButtonToClawState(js, 3, ClawState.PUT_CORAL_REEF_L4);

    // ClawState Combinate command ALGAE Reef & Processor
    setButtonToClawState(js, 7, ClawState.GET_ALGAE_REEF_HIGH);
    setButtonToClawState(js, 8, ClawState.PUT_ALGAE_PROCESSOR);
  }

  // private void player2CommandList() {
  //   final Joystick js = joystick_2;
  // }

  private void setButtonToClawState(Joystick js, int button_number, ClawState state) {
    new JoystickButton(js, button_number)
      .onTrue(new SetClawState(elevator, shaft, state)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }
}
