package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.claw.SetShaftPosition;
import frc.robot.commands.elevator.SetElevatorPower;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Claw claw = new Claw();

  private final Joystick js1 = new Joystick(Constants.OperatorConstants.Player1Port);
  private final Joystick js2 = new Joystick(Constants.OperatorConstants.Player2Port);

  public RobotContainer() {
    player1CommandList();
    player2CommandList();
  }

  private void player1CommandList() {
    final Joystick js = js1;

    // Elevator Up & Down
    new JoystickButton(js, 5).whileTrue(new SetElevatorPower(elevator, 1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 6).whileTrue(new SetElevatorPower(elevator, -1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Claw Shaft
    new JoystickButton(js, 3).whileTrue(new SetShaftPosition(claw, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 4).whileTrue(new SetShaftPosition(claw, 45).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 2).whileTrue(new SetShaftPosition(claw, 90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 1).whileTrue(new SetShaftPosition(claw, 135).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

  }

  private void player2CommandList() {
    final Joystick js = js2;

    // Elevator Up & Down
    new JoystickButton(js, 1).whileTrue(new SetElevatorPosition(elevator, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 2).whileTrue(new SetElevatorPosition(elevator, 20).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 4).whileTrue(new SetElevatorPosition(elevator, 40).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new JoystickButton(js, 3).whileTrue(new SetElevatorPosition(elevator, 60).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }
}
