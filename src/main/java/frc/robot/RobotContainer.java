package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.grapper.SetGrapperPower;
import frc.robot.commands.shaft.*;
import frc.robot.commands.swerve.*;

import frc.robot.subsystems.*;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Shaft shaft = new Shaft();
  private final Grapper grapper = new Grapper();

  private final Joystick js1 = new Joystick(Constants.OperatorConstants.Player1Port);
  private final Joystick js2 = new Joystick(Constants.OperatorConstants.Player2Port);

  public RobotContainer() {
    player1CommandList();
    player2CommandList();
  }

  private void player1CommandList() {
    final Joystick js = js1;

    // swerve
    swerve.setDefaultCommand(new JoystickSwerve(
      swerve, 
      ()->js1.getX(),
      ()->js1.getY(), 
      ()->js1.getRawAxis(4))
    );

    // Claw Grapper
    new JoystickButton(js, 5).whileTrue(new SetGrapperPower(grapper, 1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 6).whileTrue(new SetGrapperPower(grapper, -1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Combinate command
    new JoystickButton(js, 1).whileTrue(new Set_L1(elevator, shaft).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 2).whileTrue(new Set_L2(elevator, shaft).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 4).whileTrue(new Set_L3(elevator, shaft).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 3).whileTrue(new Set_L4(elevator, shaft).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  private void player2CommandList() {
    final Joystick js = js2;

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
