// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperationConstant.Reef;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Tools;

public class MoveToReef extends Command {
  private Swerve swerve;
  private Reef reef;
  private boolean fail;
  private double loss_counter;
  private double x, y, turn, tx;
  
  public MoveToReef(Swerve swerve, Reef reef, double tx) {
    this.swerve = swerve;
    this.reef = reef;
    this.fail = false;
    this.loss_counter = 0;
    this.tx = tx;
    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  @Override
  public void execute() {
    double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    System.out.println(loss_counter);
    System.out.println(fail);
    double ty;

    if(reef == Reef.Left) ty = 0.17;// left
    else ty = -0.17;// right

    if(LimelightHelpers.getTargetCount("limelight") == 0) {
      loss_counter += 1;
      return;
    }
    else {
      loss_counter = 0;
    }

    if(loss_counter > 50) {
      fail = true;
    }
    else {
      fail = false;
    }

    x = Tools.deadband((pose[2] - tx) / 1.5, 0.04);
    y = Tools.deadband((ty - pose[0]) / 0.7, 0.06);
    turn = Tools.deadband(-pose[4] / 90.0, 0.04);

    swerve.drive(x, y, turn, false);
  }

  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public boolean isFinished() {
    if(x == 0 && y == 0 && turn == 0) return true;
    return fail;
  }
}
