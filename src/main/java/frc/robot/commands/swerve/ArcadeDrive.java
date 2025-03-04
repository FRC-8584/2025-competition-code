package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants.OperationConstant;
import frc.robot.utils.Tools;

public class ArcadeDrive extends Command {
  private Swerve swerve;
  private Supplier<Double> x, y, turn;

  public ArcadeDrive(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.turn = turn;
    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    OperationConstant.axieOptimizers[0].reset();
    OperationConstant.axieOptimizers[1].reset();
    OperationConstant.axieOptimizers[2].reset();
  }

  @Override
  public void execute() {
    swerve.drive(
      OperationConstant.axieOptimizers[0].get(Tools.deadband(x.get(), 0.1)),
      OperationConstant.axieOptimizers[1].get(Tools.deadband(y.get(), 0.1)),
      OperationConstant.axieOptimizers[2].get(Tools.deadband(turn.get(), 0.1)), 
      false
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
