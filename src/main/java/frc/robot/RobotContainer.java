package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.commands.*;
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

  private final Joystick joystick_1 = new Joystick(Constants.OperatorConstants.Player1Port);
  private final Joystick joystick_2 = new Joystick(Constants.OperatorConstants.Player2Port);

  public RobotContainer() {
    elevator.setDefaultCommand(new SetElevatorPosition(elevator, elevator.getPosition()));
    shaft.setDefaultCommand(new SetShaftPosition(shaft, shaft.getPosition()));

    player1CommandList();
    player2CommandList();
  }

  private void player1CommandList() {
    final Joystick js = joystick_1;

    // swerve
    swerve.setDefaultCommand(new JoystickSwerve(
      swerve, 
      ()->joystick_1.getX(),
      ()->joystick_1.getY(), 
      ()->joystick_1.getRawAxis(4))
    );

    // Claw Grapper
    new JoystickButton(js, 5).whileTrue(new SetGrabberPower(grabber, 1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 6).whileTrue(new SetGrabberPower(grabber, -1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Combinate command
    new JoystickButton(js, 1).whileTrue(new SetClawState(elevator, shaft, 0, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 2).whileTrue(new SetClawState(elevator, shaft, 16, 35).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 4).whileTrue(new SetClawState(elevator, shaft, 35, 35).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 3).whileTrue(new SetClawState(elevator, shaft, 70, 35).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  private void player2CommandList() {
    final Joystick js = joystick_2;

    // Claw Shaft
    new JoystickButton(js, 5).whileTrue(new SetShaftPosition(shaft, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 6).whileTrue(new SetShaftPosition(shaft, 35).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 7).whileTrue(new SetShaftPosition(shaft, 130).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Elevator
    new JoystickButton(js, 1).whileTrue(new SetElevatorPosition(elevator, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 2).whileTrue(new SetElevatorPosition(elevator, 16).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 4).whileTrue(new SetElevatorPosition(elevator, 35).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 3).whileTrue(new SetElevatorPosition(elevator, 70).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }
}
