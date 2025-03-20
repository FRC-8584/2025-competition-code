package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants.OperationConstant;
import frc.robot.utils.Tools;

public class ArcadeDrive extends Command {
  private Swerve swerve;
  private Supplier<Double> x, y, turn;
  private Supplier<Boolean> isSlowDown;

  public ArcadeDrive(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn, Supplier<Boolean> isSlowDown) { 
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.turn = turn;
    this.isSlowDown = isSlowDown;
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
    double speed = isSlowDown.get() ? 0.02 : 1.0;
    OperationConstant.axieOptimizers[0].setWeight(0.2);
    OperationConstant.axieOptimizers[1].setWeight(0.2);
    OperationConstant.axieOptimizers[2].setWeight(0.3);
    swerve.drive(
      OperationConstant.axieOptimizers[0].get(Tools.deadband(x.get() * speed, speed * 0.1)),
      OperationConstant.axieOptimizers[1].get(Tools.deadband(y.get() * speed, speed * 0.1)),
      OperationConstant.axieOptimizers[2].get(Tools.deadband(turn.get() * speed, speed * 0.1)), 
      true
    );
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
