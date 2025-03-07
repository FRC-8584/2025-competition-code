package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants.OperationConstant;
import frc.robot.utils.Tools;

public class ArcadeDrive extends Command {
  private Swerve swerve;
  private boolean fieldRelative;
  private Supplier<Double> x, y, turn;
  private Supplier<Boolean> isSlowDown, changeDriveMethod;

  public ArcadeDrive(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn, Supplier<Boolean> isSlowDown, Supplier<Boolean> changeDriveMethod) { 
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.turn = turn;
    this.isSlowDown = isSlowDown;
    this.changeDriveMethod = changeDriveMethod;
    fieldRelative = true;
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
    if(changeDriveMethod.get()) {
      fieldRelative = !fieldRelative;
    }
    if(isSlowDown.get()) {
      fieldRelative = false;
      OperationConstant.axieOptimizers[0].setWeight(0.2);
      OperationConstant.axieOptimizers[1].setWeight(0.2);
      OperationConstant.axieOptimizers[2].setWeight(0.3);
    }
    else {
      OperationConstant.axieOptimizers[0].setWeight(0.1);
      OperationConstant.axieOptimizers[1].setWeight(0.1);
      OperationConstant.axieOptimizers[2].setWeight(0.15);
    }
    swerve.drive(
      OperationConstant.axieOptimizers[0].get(Tools.deadband(x.get() * (isSlowDown.get()?0.2:1.0), isSlowDown.get()?0.02:0.1)),
      OperationConstant.axieOptimizers[1].get(Tools.deadband(y.get() * (isSlowDown.get()?0.2:1.0), isSlowDown.get()?0.02:0.1)),
      OperationConstant.axieOptimizers[2].get(Tools.deadband(turn.get() * (isSlowDown.get()?0.2:1.0), isSlowDown.get()?0.02:0.1)), 
      fieldRelative
    );
    SmartDashboard.putBoolean("Swerve : Is FeildRelative", fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
