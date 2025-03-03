package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperationConstant;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Tools;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDrive extends Command {
  private Swerve swerve;
  private Supplier<Double> x, y, turn;
  public ArcadeDrive(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
    // Use addRequirements() here to declare subsystem ydependencies.
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.turn = turn;
    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    OperationConstant.axieOptimizers[0].reset();
    OperationConstant.axieOptimizers[1].reset();
    OperationConstant.axieOptimizers[2].reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
      OperationConstant.axieOptimizers[0].get(Tools.deadband(x.get(), 0.02)),
      OperationConstant.axieOptimizers[1].get(Tools.deadband(y.get(), 0.02)),
      OperationConstant.axieOptimizers[2].get(Tools.deadband(turn.get(), 0.02)), 
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
