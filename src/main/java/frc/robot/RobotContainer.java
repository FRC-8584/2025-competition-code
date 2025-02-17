package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.commands.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.grabber.*;
import frc.robot.commands.shaft.*;
import frc.robot.commands.swerve.*;

import frc.robot.subsystems.*;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Shaft shaft = new Shaft();
  private final Grabber grabber = new Grabber();
  private final Climber climber = new Climber();

  private final Joystick joystick_1 = new Joystick(Constants.OperatorConstants.Player1Port);
  private final Joystick joystick_2 = new Joystick(Constants.OperatorConstants.Player2Port);

  public RobotContainer() {
    player1CommandList();
    // player2CommandList();
  }

  private void player1CommandList() {
    final Joystick js = joystick_1;

    // swerve
    swerve.setDefaultCommand(new JoystickSwerve(
      swerve, 
      ()->js.getX(),
      ()->js.getY(), 
      ()->js.getRawAxis(4))
    );

    // Claw Grapper
    new JoystickButton(js, 5).whileTrue(new SetGrabberPower(grabber, 1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 6).whileTrue(new SetGrabberPower(grabber, -1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Combinate command
    new JoystickButton(js, 1).onTrue(new SetClawState(elevator, shaft, 0, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 2).onTrue(new SetClawState(elevator, shaft, 13, 25).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 4).onTrue(new SetClawState(elevator, shaft, 34, 25).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 3).onTrue(new SetClawState(elevator, shaft, 70, 35).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Combinate command ALGAE Reef & Processor
    new JoystickButton(js, 7).onTrue(new SetClawState(elevator, shaft, 38, 125).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 8).onTrue(new SetClawState(elevator, shaft, 14, 200).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Climber
    climber.setDefaultCommand(
      new SetClimberPower(climber, () -> js.getY())
    );
  }

  private void player2CommandList() {
    final Joystick js = joystick_2;
  }
}
